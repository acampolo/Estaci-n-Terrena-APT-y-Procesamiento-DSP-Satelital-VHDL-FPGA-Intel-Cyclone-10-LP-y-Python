library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Imagen 256x256 (8 bits/pixel, llegan en orden de lineas) -> OLED SSD1306 128x64.
--
-- 1) Reduccion: cada pixel de salida es el promedio de un bloque de 2 (horizontal)
--    x 4 (vertical) = 8 pixeles de entrada. Se cuentan las muestras: 256 por linea
--    y 256 lineas por imagen. frame_start (opcional) reinicia los contadores.
-- 2) Paso a 1 bit: umbral fijo (thresh) o dithering de Bayer 4x4 (dither_en = '1').
-- 3) Frame buffer de 128x64 bits en RAM.
-- 4) Driver SSD1306 por I2C (usa tu i2c_master): inicializa la pantalla y
--    refresca el cuadro completo de forma continua.
-- 5) Pantalla de sensores: si no llega ningun pixel durante IMG_TIMEOUT_MS (y tambien al
--    arrancar) se muestran 5 barras en un segundo frame buffer, de arriba abajo:
--       1 punto   acelerometro X   barra centrada, +-1 g
--       2 puntos  acelerometro Y   barra centrada, +-1 g
--       3 puntos  acelerometro Z   barra centrada, +-1 g
--       4 puntos  temperatura SHT31, 0 a 50 C
--       5 puntos  humedad SHT31, 0 a 100 %
--    Los marcos y la marca central se dibujan siempre, asi que la pantalla se ve aunque
--    todavia no lleguen datos. Al llegar un pixel se vuelve a la imagen.
--    sht_*/adxl_* son los mismos puertos pkt_valid / pkt_data de cada sensor_reader.
--
-- Requisito: pixel_valid separados al menos 3 ciclos de clk (con I2S son ~1000).
entity oled_img is
    generic (
        CLK_FREQ       : positive := 50_000_000;
        I2C_FREQ       : positive := 400_000;
        OLED_ADDR      : std_logic_vector(6 downto 0) := "0111100";   -- 0x3C (0x3D si el modulo lo trae asi)
        IMG_TIMEOUT_MS : positive := 2000                             -- sin pixeles durante este tiempo: mostrar sensores
    );
    port (
        clk         : in    std_logic;
        rst_n       : in    std_logic;

        pixel_in    : in    std_logic_vector(7 downto 0);
        pixel_valid : in    std_logic;
        frame_start : in    std_logic;                      -- pulso opcional, ponlo en '0' si no lo usas
        dither_en   : in    std_logic;                      -- '1' dithering Bayer, '0' umbral fijo
        thresh      : in    std_logic_vector(7 downto 0);   -- umbral para el modo fijo

        -- Si no se conectan en el top-level quedan en 0 y las barras se ven vacias
        sht_valid   : in    std_logic := '0';                                  -- pkt_valid del sensor_reader del SHT31
        sht_data    : in    std_logic_vector(127 downto 0) := (others => '0'); -- pkt_data del SHT31
        adxl_valid  : in    std_logic := '0';                                  -- pkt_valid del sensor_reader del ADXL345
        adxl_data   : in    std_logic_vector(127 downto 0) := (others => '0'); -- pkt_data del ADXL345

        scl         : inout std_logic;
        sda         : inout std_logic;
        nack_err    : out   std_logic;                      -- '1' si la pantalla no responde (sin ACK)
        modo_sensores : out std_logic                       -- '1' mientras se muestran las barras
    );
end entity;

architecture rtl of oled_img is

    -- Comandos del SSD1306: 25 de inicializacion y 6 para fijar la ventana
    type rom_t is array (0 to 30) of std_logic_vector(7 downto 0);
    constant CMD_ROM : rom_t := (
        x"AE",                -- pantalla apagada
        x"D5", x"80",         -- divisor de reloj
        x"A8", x"3F",         -- multiplex 64 filas
        x"D3", x"00",         -- offset de pantalla
        x"40",                -- linea de inicio 0
        x"8D", x"14",         -- charge pump activado
        x"20", x"00",         -- direccionamiento horizontal
        x"A1",                -- remapeo de segmentos
        x"C8",                -- direccion de barrido COM invertida
        x"DA", x"12",         -- configuracion de pines COM
        x"81", x"CF",         -- contraste
        x"D9", x"F1",         -- precarga
        x"DB", x"40",         -- nivel VCOMH
        x"A4",                -- mostrar contenido de la RAM
        x"A6",                -- polaridad normal
        x"AF",                -- pantalla encendida
        x"21", x"00", x"7F",  -- columnas 0..127
        x"22", x"00", x"07"   -- paginas 0..7
    );
    constant INIT_LEN : integer := 25;
    constant WIN_LEN  : integer := 6;
    constant DATA_LEN : integer := 1024;

    -- Matriz de Bayer 4x4
    type bayer_t is array (0 to 15) of integer range 0 to 15;
    constant BAYER : bayer_t := (0, 8, 2, 10,
                                 12, 4, 14, 6,
                                 3, 11, 1, 9,
                                 15, 7, 13, 5);

    -- Reduccion
    type acc_t is array (0 to 127) of unsigned(10 downto 0);
    signal acc       : acc_t := (others => (others => '0'));
    signal acc_q     : unsigned(10 downto 0) := (others => '0');
    signal acc_we    : std_logic := '0';
    signal acc_waddr : unsigned(6 downto 0) := (others => '0');
    signal acc_wdata : unsigned(10 downto 0) := (others => '0');

    type q_t is (Q_IDLE, Q_ACC);
    signal qst   : q_t := Q_IDLE;
    signal x_cnt : unsigned(7 downto 0) := (others => '0');
    signal y_cnt : unsigned(7 downto 0) := (others => '0');
    signal px_r  : unsigned(7 downto 0) := (others => '0');

    -- Frame buffer de la imagen: direccion = fila*128 + columna
    type fb_t is array (0 to 8191) of std_logic;
    signal fb       : fb_t := (others => '0');
    signal fb_we    : std_logic := '0';
    signal fb_waddr : unsigned(12 downto 0) := (others => '0');
    signal fb_wdata : std_logic := '0';
    signal rd_addr  : unsigned(12 downto 0) := (others => '0');
    signal rd_q1    : std_logic := '0';
    signal rd_q2    : std_logic := '0';
    signal rd_q     : std_logic := '0';

    -- Frame buffer de la pantalla de sensores
    signal fb2       : fb_t := (others => '0');
    signal fb2_we    : std_logic := '0';
    signal fb2_waddr : unsigned(12 downto 0) := (others => '0');
    signal fb2_wdata : std_logic := '0';

    -- Modo de pantalla
    constant TIMEOUT_CLKS : positive := (CLK_FREQ / 1000) * IMG_TIMEOUT_MS;
    signal idle_cnt      : integer range 0 to TIMEOUT_CLKS := TIMEOUT_CLKS;
    signal show_sensors  : std_logic := '1';

    -- Largo en pixeles de cada barra
    signal len_x : integer range -60 to 60 := 0;
    signal len_y : integer range -60 to 60 := 0;
    signal len_z : integer range -60 to 60 := 0;
    signal len_t : integer range 0 to 120 := 0;
    signal len_h : integer range 0 to 120 := 0;

    -- Recorrido del frame buffer de sensores
    signal sx : integer range 0 to 127 := 0;
    signal sy : integer range 0 to 63 := 0;

    function clamp(v, lo, hi : integer) return integer is
    begin
        if v < lo then
            return lo;
        elsif v > hi then
            return hi;
        else
            return v;
        end if;
    end function;

    -- Maestro I2C
    signal m_valid, m_start, m_stop, m_write : std_logic := '0';
    signal m_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal m_ready, m_done, m_ackerr : std_logic;

    -- Driver
    type d_t is (S_PWRUP, S_SEND_ADDR, S_WAIT_ADDR, S_SEND_CTRL, S_WAIT_CTRL,
                 S_LOAD, S_FA, S_FB, S_FC, S_SEND_BYTE, S_WAIT_BYTE, S_END, S_ERRWAIT);
    type mode_t is (MD_INIT, MD_WIN, MD_DATA);
    signal dst      : d_t := S_PWRUP;
    signal mode     : mode_t := MD_INIT;
    signal idx      : integer range 0 to DATA_LEN := 0;
    signal nbytes   : integer range 0 to DATA_LEN := INIT_LEN;
    signal bit_i    : integer range 0 to 7 := 0;
    signal cur_byte : std_logic_vector(7 downto 0) := (others => '0');
    signal err      : std_logic := '0';
    signal wait_cnt : integer range 0 to CLK_FREQ / 10 := 0;

begin

    u_i2c : entity work.i2c_master
        generic map (CLK_FREQ => CLK_FREQ, I2C_FREQ => I2C_FREQ)
        port map (
            clk       => clk,
            rst_n     => rst_n,
            cmd_valid => m_valid,
            cmd_start => m_start,
            cmd_stop  => m_stop,
            cmd_write => m_write,
            cmd_read  => '0',
            cmd_ack   => '0',
            wr_data   => m_data,
            cmd_ready => m_ready,
            done      => m_done,
            rd_data   => open,
            ack_err   => m_ackerr,
            scl       => scl,
            sda       => sda
        );

    nbytes <= INIT_LEN when mode = MD_INIT else
              WIN_LEN  when mode = MD_WIN  else
              DATA_LEN;

    modo_sensores <= show_sensors;
    rd_q <= rd_q2 when show_sensors = '1' else rd_q1;

    ---------------------------------------------------------------------------
    -- RAMs (sin reset para que se infieran como bloques de memoria)
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if acc_we = '1' then
                acc(to_integer(acc_waddr)) <= acc_wdata;
            end if;
            acc_q <= acc(to_integer(x_cnt(7 downto 1)));
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            if fb_we = '1' then
                fb(to_integer(fb_waddr)) <= fb_wdata;
            end if;
            rd_q1 <= fb(to_integer(rd_addr));
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            if fb2_we = '1' then
                fb2(to_integer(fb2_waddr)) <= fb2_wdata;
            end if;
            rd_q2 <= fb2(to_integer(rd_addr));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Modo de pantalla: imagen mientras lleguen pixeles, sensores si dejan de llegar
    ---------------------------------------------------------------------------
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            idle_cnt     <= TIMEOUT_CLKS;
            show_sensors <= '1';
        elsif rising_edge(clk) then
            if pixel_valid = '1' then
                idle_cnt     <= 0;
                show_sensors <= '0';
            elsif idle_cnt < TIMEOUT_CLKS then
                idle_cnt <= idle_cnt + 1;
            else
                show_sensors <= '1';
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Largo de cada barra a partir de los paquetes de sensor
    --   ADXL345: X, Y, Z de 16 bits con signo, byte bajo primero, 256 LSB por g
    --   SHT31:   T_msb T_lsb CRC H_msb H_lsb CRC
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable raw : integer;
    begin
        if rst_n = '0' then
            len_x <= 0;
            len_y <= 0;
            len_z <= 0;
            len_t <= 0;
            len_h <= 0;
        elsif rising_edge(clk) then
            if adxl_valid = '1' then
                raw := to_integer(signed(adxl_data(15 downto 0)));
                len_x <= clamp((raw * 15) / 64, -60, 60);        -- 256 LSB = 60 pixeles
                raw := to_integer(signed(adxl_data(31 downto 16)));
                len_y <= clamp((raw * 15) / 64, -60, 60);
                raw := to_integer(signed(adxl_data(47 downto 32)));
                len_z <= clamp((raw * 15) / 64, -60, 60);
            end if;
            if sht_valid = '1' then
                raw := to_integer(unsigned(sht_data(7 downto 0)) & unsigned(sht_data(15 downto 8)));
                len_t <= clamp((raw * 420) / 65536 - 108, 0, 120);   -- 0 a 50 C en 120 pixeles
                raw := to_integer(unsigned(sht_data(31 downto 24)) & unsigned(sht_data(39 downto 32)));
                len_h <= clamp((raw * 120) / 65536, 0, 120);         -- 0 a 100 % en 120 pixeles
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Dibujo de las barras: recorre los 8192 pixeles una y otra vez
    -- Cada barra ocupa 10 filas (3, 15, 27, 39 y 51 son las filas de inicio)
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable b, r, lv : integer;
        variable on_px    : boolean;
    begin
        if rst_n = '0' then
            sx        <= 0;
            sy        <= 0;
            fb2_we    <= '0';
            fb2_waddr <= (others => '0');
            fb2_wdata <= '0';

        elsif rising_edge(clk) then
            b := -1;
            r := 0;
            if sy >= 3 and sy <= 12 then
                b := 0; r := sy - 3;
            elsif sy >= 15 and sy <= 24 then
                b := 1; r := sy - 15;
            elsif sy >= 27 and sy <= 36 then
                b := 2; r := sy - 27;
            elsif sy >= 39 and sy <= 48 then
                b := 3; r := sy - 39;
            elsif sy >= 51 and sy <= 60 then
                b := 4; r := sy - 51;
            end if;

            on_px := false;
            if b >= 0 then
                -- marco de la barra
                if (r = 0 or r = 9) and sx >= 3 and sx <= 124 then
                    on_px := true;
                end if;
                if sx = 3 or sx = 124 then
                    on_px := true;
                end if;

                -- relleno
                case b is
                    when 0 => lv := len_x;
                    when 1 => lv := len_y;
                    when 2 => lv := len_z;
                    when 3 => lv := len_t;
                    when others => lv := len_h;
                end case;
                if r >= 2 and r <= 7 then
                    if b <= 2 then                              -- ejes: barra centrada en x = 64
                        if lv >= 0 then
                            if sx >= 64 and sx < 64 + lv then
                                on_px := true;
                            end if;
                        else
                            if sx >= 64 + lv and sx < 64 then
                                on_px := true;
                            end if;
                        end if;
                    else                                        -- temperatura y humedad: desde la izquierda
                        if sx >= 4 and sx < 4 + lv then
                            on_px := true;
                        end if;
                    end if;
                end if;

                -- marca central de los ejes del acelerometro
                if b <= 2 and sx = 64 then
                    on_px := true;
                end if;

                -- puntos de identificacion en el borde izquierdo: b + 1 puntos
                if sx = 0 and (r = 0 or r = 2 or r = 4 or r = 6 or r = 8) and r / 2 <= b then
                    on_px := true;
                end if;
            end if;

            fb2_we    <= '1';
            fb2_waddr <= to_unsigned(sy, 6) & to_unsigned(sx, 7);
            if on_px then
                fb2_wdata <= '1';
            else
                fb2_wdata <= '0';
            end if;

            if sx = 127 then
                sx <= 0;
                if sy = 63 then
                    sy <= 0;
                else
                    sy <= sy + 1;
                end if;
            else
                sx <= sx + 1;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Reduccion 256x256 -> 128x64 y paso a 1 bit
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable base, sum : unsigned(10 downto 0);
        variable avg, thr  : unsigned(7 downto 0);
        variable bidx      : integer range 0 to 15;
    begin
        if rst_n = '0' then
            qst      <= Q_IDLE;
            x_cnt    <= (others => '0');
            y_cnt    <= (others => '0');
            px_r     <= (others => '0');
            acc_we   <= '0';
            acc_waddr <= (others => '0');
            acc_wdata <= (others => '0');
            fb_we    <= '0';
            fb_waddr <= (others => '0');
            fb_wdata <= '0';

        elsif rising_edge(clk) then
            acc_we <= '0';
            fb_we  <= '0';

            if frame_start = '1' then
                x_cnt <= (others => '0');
                y_cnt <= (others => '0');
                qst   <= Q_IDLE;
            else
                case qst is

                    when Q_IDLE =>
                        if pixel_valid = '1' then
                            px_r <= unsigned(pixel_in);
                            qst  <= Q_ACC;
                        end if;

                    when Q_ACC =>
                        -- primer pixel del bloque: empezar la suma de cero
                        if x_cnt(0) = '0' and y_cnt(1 downto 0) = "00" then
                            base := (others => '0');
                        else
                            base := acc_q;
                        end if;
                        sum := base + resize(px_r, 11);
                        acc_we    <= '1';
                        acc_waddr <= x_cnt(7 downto 1);
                        acc_wdata <= sum;

                        -- ultimo pixel del bloque (8 en total): promedio = suma / 8
                        if x_cnt(0) = '1' and y_cnt(1 downto 0) = "11" then
                            avg := sum(10 downto 3);
                            if dither_en = '1' then
                                bidx := to_integer(y_cnt(3 downto 2) & x_cnt(2 downto 1));
                                thr  := to_unsigned(BAYER(bidx) * 16 + 8, 8);
                            else
                                thr  := unsigned(thresh);
                            end if;
                            fb_waddr <= y_cnt(7 downto 2) & x_cnt(7 downto 1);
                            if avg > thr then
                                fb_wdata <= '1';
                            else
                                fb_wdata <= '0';
                            end if;
                            fb_we <= '1';
                        end if;

                        x_cnt <= x_cnt + 1;
                        if x_cnt = 255 then
                            y_cnt <= y_cnt + 1;
                        end if;
                        qst <= Q_IDLE;

                end case;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Driver SSD1306
    --   INIT : START, 0x78, 0x00, 25 comandos, STOP
    --   WIN  : START, 0x78, 0x00, 6 comandos (ventana 128x8 paginas), STOP
    --   DATA : START, 0x78, 0x40, 1024 bytes de imagen, STOP
    -- Despues de INIT repite WIN, DATA, WIN, DATA... (refresco continuo)
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable a : unsigned(9 downto 0);
    begin
        if rst_n = '0' then
            dst      <= S_PWRUP;
            mode     <= MD_INIT;
            idx      <= 0;
            bit_i    <= 0;
            cur_byte <= (others => '0');
            err      <= '0';
            nack_err <= '0';
            wait_cnt <= 0;
            rd_addr  <= (others => '0');
            m_valid  <= '0';
            m_start  <= '0';
            m_stop   <= '0';
            m_write  <= '0';
            m_data   <= (others => '0');

        elsif rising_edge(clk) then
            m_valid <= '0';

            case dst is

                when S_PWRUP =>                         -- 100 ms para que la pantalla arranque
                    if wait_cnt = CLK_FREQ / 10 then
                        wait_cnt <= 0;
                        mode     <= MD_INIT;
                        dst      <= S_SEND_ADDR;
                    else
                        wait_cnt <= wait_cnt + 1;
                    end if;

                when S_SEND_ADDR =>
                    if m_ready = '1' then
                        m_start <= '1';
                        m_stop  <= '0';
                        m_write <= '1';
                        m_data  <= OLED_ADDR & '0';     -- direccion + bit de escritura
                        m_valid <= '1';
                        dst     <= S_WAIT_ADDR;
                    end if;

                when S_WAIT_ADDR =>
                    if m_done = '1' then
                        if m_ackerr = '1' then err <= '1'; end if;
                        dst <= S_SEND_CTRL;
                    end if;

                when S_SEND_CTRL =>
                    if m_ready = '1' then
                        m_start <= '0';
                        m_stop  <= '0';
                        m_write <= '1';
                        if mode = MD_DATA then
                            m_data <= x"40";            -- lo que sigue son datos
                        else
                            m_data <= x"00";            -- lo que sigue son comandos
                        end if;
                        m_valid <= '1';
                        dst     <= S_WAIT_CTRL;
                    end if;

                when S_WAIT_CTRL =>
                    if m_done = '1' then
                        if m_ackerr = '1' then err <= '1'; end if;
                        idx   <= 0;
                        bit_i <= 0;
                        if mode = MD_DATA then
                            dst <= S_FA;
                        else
                            dst <= S_LOAD;
                        end if;
                    end if;

                when S_LOAD =>                          -- siguiente comando de la ROM
                    if mode = MD_INIT then
                        cur_byte <= CMD_ROM(idx);
                    else
                        cur_byte <= CMD_ROM(INIT_LEN + idx);
                    end if;
                    dst <= S_SEND_BYTE;

                -- Armar un byte de pagina: 8 bits verticales, LSB arriba
                when S_FA =>
                    a := to_unsigned(idx, 10);          -- a(9:7) = pagina, a(6:0) = columna
                    rd_addr <= a(9 downto 7) & to_unsigned(bit_i, 3) & a(6 downto 0);
                    dst <= S_FB;

                when S_FB =>                            -- latencia de la RAM
                    dst <= S_FC;

                when S_FC =>
                    cur_byte(bit_i) <= rd_q;
                    if bit_i = 7 then
                        bit_i <= 0;
                        dst   <= S_SEND_BYTE;
                    else
                        bit_i <= bit_i + 1;
                        dst   <= S_FA;
                    end if;

                when S_SEND_BYTE =>
                    if m_ready = '1' then
                        m_start <= '0';
                        m_write <= '1';
                        m_data  <= cur_byte;
                        if idx = nbytes - 1 then
                            m_stop <= '1';
                        else
                            m_stop <= '0';
                        end if;
                        m_valid <= '1';
                        dst     <= S_WAIT_BYTE;
                    end if;

                when S_WAIT_BYTE =>
                    if m_done = '1' then
                        if m_ackerr = '1' then err <= '1'; end if;
                        if idx = nbytes - 1 then
                            dst <= S_END;
                        else
                            idx <= idx + 1;
                            if mode = MD_DATA then
                                dst <= S_FA;
                            else
                                dst <= S_LOAD;
                            end if;
                        end if;
                    end if;

                when S_END =>
                    if err = '1' then                   -- sin ACK: reintentar desde cero
                        nack_err <= '1';
                        wait_cnt <= 0;
                        dst      <= S_ERRWAIT;
                    else
                        case mode is
                            when MD_INIT => mode <= MD_WIN;
                            when MD_WIN  => mode <= MD_DATA;
                            when MD_DATA => mode <= MD_WIN;
                                           nack_err <= '0';
                        end case;
                        dst <= S_SEND_ADDR;
                    end if;

                when S_ERRWAIT =>
                    if wait_cnt = CLK_FREQ / 10 then
                        wait_cnt <= 0;
                        err      <= '0';
                        mode     <= MD_INIT;
                        dst      <= S_SEND_ADDR;
                    else
                        wait_cnt <= wait_cnt + 1;
                    end if;

            end case;
        end if;
    end process;

end architecture;