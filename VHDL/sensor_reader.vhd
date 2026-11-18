library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Lector de un sensor I2C. Se instancia una vez por sensor (cada uno tiene su bus) y
-- trae dentro su propio i2c_master. Segun el generic SENSOR ejecuta una secuencia distinta:
--
--   SENSOR = 0  SHT31  (0x44): comando 0x2400, espera 20 ms, lee 6 bytes
--                       paquete: T_msb T_lsb T_crc H_msb H_lsb H_crc
--   SENSOR = 1  ADXL345 (0x53): activa medicion (reg 0x2D = 0x08), lee 6 bytes desde 0x32
--                       paquete: X_lsb X_msb Y_lsb Y_msb Z_lsb Z_msb   (complemento a 2)
--   SENSOR = 2  MS5611 (0x76 con CSB = '1'): reset, lee la PROM (C1..C6) y despues mide
--                       presion (D1) y temperatura (D2) con OSR = 4096
--                       paquete de calibracion (una vez, pkt_kind = '1'): C1..C6, 12 bytes
--                       paquete de medida (pkt_kind = '0'): D1 (3 bytes) D2 (3 bytes)
--                       La compensacion de 64 bits se hace en la laptop.
--
-- Cada vez que hay un paquete nuevo: pkt_valid = '1' durante un ciclo, y pkt_data
-- (byte i en los bits 8*i+7 .. 8*i) y pkt_len quedan estables hasta el siguiente paquete.
-- Si algun byte no recibe ACK, nack_err = '1' y el sensor se reinicia a los 100 ms.
--
-- Pines de la placa: ADXL_CS = '1' (modo I2C), ADXL_SDO = '0' (direccion 0x53),
-- MS_PS = '1' (modo I2C), MS_CSB = '1'. Los maneja la FPGA, forzalos en el top.
entity sensor_reader is
    generic (
        CLK_FREQ  : positive := 50_000_000;
        I2C_FREQ  : positive := 400_000;
        SENSOR    : natural range 0 to 2 := 0;
        I2C_ADDR  : std_logic_vector(6 downto 0) := "1000100";  -- SHT31 0x44, ADXL345 "1010011" (0x53), MS5611 "1110110" (0x76)
        PERIOD_MS : positive range 1 to 60000 := 100            -- tiempo entre medidas
    );
    port (
        clk       : in    std_logic;
        rst_n     : in    std_logic;

        pkt_valid : out   std_logic;
        pkt_kind  : out   std_logic;                        -- '0' medida, '1' calibracion
        pkt_len   : out   unsigned(4 downto 0);             -- bytes validos del paquete
        pkt_data  : out   std_logic_vector(127 downto 0);

        scl       : inout std_logic;
        sda       : inout std_logic;
        nack_err  : out   std_logic
    );
end entity;

architecture rtl of sensor_reader is

    -- Una operacion: escribir 0..2 bytes, leer 0..6 bytes, esperar N ms
    type op_t is record
        wr_n : integer range 0 to 2;
        wb0  : std_logic_vector(7 downto 0);
        wb1  : std_logic_vector(7 downto 0);
        rd_n : integer range 0 to 6;
        rs   : boolean;                  -- true: START repetido entre escritura y lectura
        wms  : integer range 0 to 31;    -- espera al terminar
    end record;
    type op_arr_t is array (0 to 15) of op_t;
    type cfg_t is record
        ops        : op_arr_t;
        n_init     : integer range 0 to 15;   -- operaciones de inicio (se ejecutan una vez)
        n_loop     : integer range 1 to 15;   -- operaciones que se repiten
        init_bytes : integer range 0 to 16;
    end record;

    constant NOP_OP : op_t := (0, x"00", x"00", 0, false, 0);

    function mk(wr_n : integer; wb0, wb1 : std_logic_vector(7 downto 0);
                rd_n : integer; rs : boolean; wms : integer) return op_t is
        variable o : op_t;
    begin
        o.wr_n := wr_n; o.wb0 := wb0; o.wb1 := wb1;
        o.rd_n := rd_n; o.rs := rs;   o.wms := wms;
        return o;
    end function;

    function get_cfg(s : natural) return cfg_t is
        variable c : cfg_t;
    begin
        c.ops := (others => NOP_OP);
        case s is
            when 0 =>   -- SHT31
                c.n_init := 0; c.n_loop := 2; c.init_bytes := 0;
                c.ops(0) := mk(2, x"24", x"00", 0, false, 20);   -- medida unica, sin clock stretching
                c.ops(1) := mk(0, x"00", x"00", 6, false, 0);
            when 1 =>   -- ADXL345
                c.n_init := 1; c.n_loop := 1; c.init_bytes := 0;
                c.ops(0) := mk(2, x"2D", x"08", 0, false, 0);    -- POWER_CTL: medir
                c.ops(1) := mk(1, x"32", x"00", 6, true, 0);     -- DATAX0..DATAZ1
            when others =>  -- MS5611
                c.n_init := 7; c.n_loop := 4; c.init_bytes := 12;
                c.ops(0) := mk(1, x"1E", x"00", 0, false, 5);    -- reset
                c.ops(1) := mk(1, x"A2", x"00", 2, false, 0);    -- C1
                c.ops(2) := mk(1, x"A4", x"00", 2, false, 0);    -- C2
                c.ops(3) := mk(1, x"A6", x"00", 2, false, 0);    -- C3
                c.ops(4) := mk(1, x"A8", x"00", 2, false, 0);    -- C4
                c.ops(5) := mk(1, x"AA", x"00", 2, false, 0);    -- C5
                c.ops(6) := mk(1, x"AC", x"00", 2, false, 0);    -- C6
                c.ops(7) := mk(1, x"48", x"00", 0, false, 10);   -- convertir presion, OSR 4096
                c.ops(8) := mk(1, x"00", x"00", 3, false, 0);    -- leer D1
                c.ops(9) := mk(1, x"58", x"00", 0, false, 10);   -- convertir temperatura
                c.ops(10) := mk(1, x"00", x"00", 3, false, 0);   -- leer D2
        end case;
        return c;
    end function;

    constant CFG     : cfg_t := get_cfg(SENSOR);
    constant CLKS_MS : positive := CLK_FREQ / 1000;

    type st_t is (S_PWRUP, S_OP, S_WADDR, S_WBYTE, S_RADDR, S_RBYTE, S_ISSUE, S_DONE,
                  S_OPWAIT, S_WAITMS, S_NEXT, S_PUB, S_PERIOD, S_ERR);
    type kind_t is (K_WADDR, K_WBYTE, K_RADDR, K_RBYTE);
    type rbuf_t is array (0 to 15) of std_logic_vector(7 downto 0);

    signal st       : st_t := S_PWRUP;
    signal kind     : kind_t := K_WADDR;
    signal opi      : integer range 0 to 15 := 0;
    signal in_init  : std_logic := '0';
    signal wi       : integer range 0 to 1 := 0;
    signal ri       : integer range 0 to 5 := 0;
    signal rcnt     : integer range 0 to 16 := 0;
    signal rbuf     : rbuf_t := (others => (others => '0'));
    signal err      : std_logic := '0';
    signal pub_kind : std_logic := '0';

    signal ms_cnt   : integer range 0 to CLKS_MS - 1 := 0;
    signal ms_tick  : std_logic := '0';
    signal wms_left : integer range 0 to 60000 := 0;

    signal m_valid, m_start, m_stop, m_write, m_read, m_ack : std_logic := '0';
    signal m_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal m_ready, m_done, m_ackerr : std_logic;
    signal m_rd     : std_logic_vector(7 downto 0);

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
            cmd_read  => m_read,
            cmd_ack   => m_ack,
            wr_data   => m_data,
            cmd_ready => m_ready,
            done      => m_done,
            rd_data   => m_rd,
            ack_err   => m_ackerr,
            scl       => scl,
            sda       => sda
        );

    process(clk, rst_n)
        variable op   : op_t;
        variable last : boolean;
    begin
        if rst_n = '0' then
            st        <= S_PWRUP;
            kind      <= K_WADDR;
            opi       <= 0;
            if CFG.n_init > 0 then in_init <= '1'; else in_init <= '0'; end if;
            wi        <= 0;
            ri        <= 0;
            rcnt      <= 0;
            err       <= '0';
            pub_kind  <= '0';
            ms_cnt    <= 0;
            ms_tick   <= '0';
            wms_left  <= 50;                       -- 50 ms de arranque de los sensores
            m_valid   <= '0';
            m_start   <= '0';
            m_stop    <= '0';
            m_write   <= '0';
            m_read    <= '0';
            m_ack     <= '0';
            m_data    <= (others => '0');
            pkt_valid <= '0';
            pkt_kind  <= '0';
            pkt_len   <= (others => '0');
            pkt_data  <= (others => '0');
            nack_err  <= '0';

        elsif rising_edge(clk) then
            m_valid   <= '0';
            pkt_valid <= '0';

            -- Base de tiempo de 1 ms
            ms_tick <= '0';
            if ms_cnt = CLKS_MS - 1 then
                ms_cnt  <= 0;
                ms_tick <= '1';
            else
                ms_cnt <= ms_cnt + 1;
            end if;

            op := CFG.ops(opi);

            case st is

                when S_PWRUP =>
                    if ms_tick = '1' then
                        if wms_left <= 1 then
                            st <= S_OP;
                        else
                            wms_left <= wms_left - 1;
                        end if;
                    end if;

                when S_OP =>                                -- decidir por donde empieza la operacion
                    if op.wr_n > 0 then
                        st <= S_WADDR;
                    elsif op.rd_n > 0 then
                        st <= S_RADDR;
                    else
                        st <= S_OPWAIT;
                    end if;

                -- Preparacion de comandos para el i2c_master
                when S_WADDR =>                             -- START + direccion con bit de escritura
                    m_start <= '1'; m_stop <= '0'; m_write <= '1'; m_read <= '0'; m_ack <= '0';
                    m_data  <= I2C_ADDR & '0';
                    kind    <= K_WADDR;
                    st      <= S_ISSUE;

                when S_WBYTE =>
                    last := (wi = op.wr_n - 1);
                    m_start <= '0'; m_write <= '1'; m_read <= '0'; m_ack <= '0';
                    if wi = 0 then m_data <= op.wb0; else m_data <= op.wb1; end if;
                    if last and (op.rd_n = 0 or not op.rs) then
                        m_stop <= '1';
                    else
                        m_stop <= '0';
                    end if;
                    kind <= K_WBYTE;
                    st   <= S_ISSUE;

                when S_RADDR =>                             -- (re)START + direccion con bit de lectura
                    m_start <= '1'; m_stop <= '0'; m_write <= '1'; m_read <= '0'; m_ack <= '0';
                    m_data  <= I2C_ADDR & '1';
                    kind    <= K_RADDR;
                    st      <= S_ISSUE;

                when S_RBYTE =>                             -- ACK en todos menos en el ultimo byte
                    last := (ri = op.rd_n - 1);
                    m_start <= '0'; m_write <= '0'; m_read <= '1';
                    if last then
                        m_ack <= '0'; m_stop <= '1';
                    else
                        m_ack <= '1'; m_stop <= '0';
                    end if;
                    kind <= K_RBYTE;
                    st   <= S_ISSUE;

                when S_ISSUE =>
                    if m_ready = '1' then
                        m_valid <= '1';
                        st      <= S_DONE;
                    end if;

                when S_DONE =>
                    if m_done = '1' then
                        case kind is
                            when K_WADDR =>
                                if m_ackerr = '1' then err <= '1'; end if;
                                wi <= 0;
                                st <= S_WBYTE;

                            when K_WBYTE =>
                                if m_ackerr = '1' then err <= '1'; end if;
                                if wi < op.wr_n - 1 then
                                    wi <= wi + 1;
                                    st <= S_WBYTE;
                                elsif op.rd_n > 0 then
                                    st <= S_RADDR;
                                else
                                    st <= S_OPWAIT;
                                end if;

                            when K_RADDR =>
                                if m_ackerr = '1' then err <= '1'; end if;
                                ri <= 0;
                                st <= S_RBYTE;

                            when K_RBYTE =>
                                if rcnt < 16 then
                                    rbuf(rcnt) <= m_rd;
                                    rcnt       <= rcnt + 1;
                                end if;
                                if ri < op.rd_n - 1 then
                                    ri <= ri + 1;
                                    st <= S_RBYTE;
                                else
                                    st <= S_OPWAIT;
                                end if;
                        end case;
                    end if;

                when S_OPWAIT =>                            -- espera tras la operacion (conversion del sensor)
                    if op.wms > 0 then
                        ms_cnt   <= 0;
                        wms_left <= op.wms;
                        st       <= S_WAITMS;
                    else
                        st <= S_NEXT;
                    end if;

                when S_WAITMS =>
                    if ms_tick = '1' then
                        if wms_left <= 1 then
                            st <= S_NEXT;
                        else
                            wms_left <= wms_left - 1;
                        end if;
                    end if;

                when S_NEXT =>
                    if err = '1' then                       -- hubo NACK: reiniciar el sensor
                        nack_err <= '1';
                        ms_cnt   <= 0;
                        wms_left <= 100;
                        st       <= S_ERR;
                    elsif in_init = '1' and opi = CFG.n_init - 1 then
                        in_init <= '0';                     -- fin del inicio
                        opi     <= CFG.n_init;
                        if CFG.init_bytes > 0 then
                            pub_kind <= '1';
                            st       <= S_PUB;
                        else
                            rcnt <= 0;
                            st   <= S_OP;
                        end if;
                    elsif in_init = '0' and opi = CFG.n_init + CFG.n_loop - 1 then
                        pub_kind <= '0';                    -- fin de la secuencia periodica
                        st       <= S_PUB;
                    else
                        opi <= opi + 1;
                        st  <= S_OP;
                    end if;

                when S_PUB =>
                    for i in 0 to 15 loop
                        pkt_data(8 * i + 7 downto 8 * i) <= rbuf(i);
                    end loop;
                    pkt_len   <= to_unsigned(rcnt, 5);
                    pkt_kind  <= pub_kind;
                    pkt_valid <= '1';
                    rcnt      <= 0;
                    if pub_kind = '1' then                  -- calibracion: seguir con las medidas
                        st <= S_OP;
                    else
                        nack_err <= '0';
                        opi      <= CFG.n_init;
                        ms_cnt   <= 0;
                        wms_left <= PERIOD_MS;
                        st       <= S_PERIOD;
                    end if;

                when S_PERIOD =>
                    if ms_tick = '1' then
                        if wms_left <= 1 then
                            st <= S_OP;
                        else
                            wms_left <= wms_left - 1;
                        end if;
                    end if;

                when S_ERR =>
                    if ms_tick = '1' then
                        if wms_left <= 1 then
                            err  <= '0';
                            opi  <= 0;
                            rcnt <= 0;
                            if CFG.n_init > 0 then in_init <= '1'; else in_init <= '0'; end if;
                            st   <= S_OP;
                        else
                            wms_left <= wms_left - 1;
                        end if;
                    end if;

            end case;
        end if;
    end process;

end architecture;