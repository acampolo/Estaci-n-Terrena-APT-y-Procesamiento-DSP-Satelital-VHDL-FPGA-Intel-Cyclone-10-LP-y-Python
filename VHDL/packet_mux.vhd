library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Empaquetador de la trama serie hacia la laptop. Todo paquete empieza con A5 5A.
--
--   Linea de imagen   A5 5A A1 idx + 256 pixeles                    260 bytes
--   SHT31             A5 5A B1 + 6 bytes + checksum                  10 bytes
--   ADXL345           A5 5A B2 + 6 bytes + checksum                  10 bytes
--   MS5611 medida     A5 5A B3 + D1(3) D2(3) + checksum              10 bytes
--   MS5611 calibrac.  A5 5A B4 + C1..C6 (12 bytes) + checksum        16 bytes
--
-- checksum = XOR del byte de tipo y de todos los bytes de datos.
--
-- Reglas:
--  * Los paquetes de sensor solo salen ENTRE lineas, nunca en medio de una.
--  * Los pixeles se cuentan aqui igual que en oled_img (256 por linea, 256 lineas).
--    frame_start reinicia los contadores; usa la misma senal en ambos bloques.
--    Si frame_start llega a mitad de una linea, la linea en curso se completa con ceros
--    para que la laptop siempre reciba 256 pixeles por cabecera.
--  * La cabecera de linea solo se envia cuando ya hay un pixel esperando.
--  * Cada sensor guarda su ultimo paquete; si llegan dos antes de poder enviarlos, sale el mas reciente.
--  * La calibracion del MS5611 se reenvia cada CAL_PERIOD_S segundos.
--  * 'descartado' queda en '1' si el FIFO de pixeles se llenó (depuracion).
entity packet_mux is
    generic (
        CLK_FREQ     : positive := 50_000_000;
        CAL_PERIOD_S : positive range 1 to 20 := 10
    );
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;

        -- pixeles (salida de hilbert)
        pixel_valid : in  std_logic;
        pixel_in    : in  std_logic_vector(7 downto 0);
        frame_start : in  std_logic;                       -- opcional, '0' si no se usa

        -- sensores (puertos pkt_* de cada sensor_reader)
        sht_valid   : in  std_logic;
        sht_data    : in  std_logic_vector(127 downto 0);
        adxl_valid  : in  std_logic;
        adxl_data   : in  std_logic_vector(127 downto 0);
        ms_valid    : in  std_logic;
        ms_kind     : in  std_logic;                       -- '0' medida, '1' calibracion
        ms_data     : in  std_logic_vector(127 downto 0);

        -- hacia uart_tx
        tx_data     : out std_logic_vector(7 downto 0);
        tx_valid    : out std_logic;
        tx_ready    : in  std_logic;

        descartado  : out std_logic
    );
end entity;

architecture rtl of packet_mux is

    constant FIFO_DEPTH : positive := 64;
    constant CAL_CLKS   : positive := CLK_FREQ * CAL_PERIOD_S;

    -- FIFO de pixeles: bit 16 = primer pixel de la linea, 15..8 = indice de linea, 7..0 = pixel
    type fifo_t is array (0 to FIFO_DEPTH - 1) of std_logic_vector(16 downto 0);
    signal fifo   : fifo_t := (others => (others => '0'));
    signal wr_ptr : unsigned(5 downto 0) := (others => '0');
    signal rd_ptr : unsigned(5 downto 0) := (others => '0');
    signal cnt    : integer range 0 to FIFO_DEPTH := 0;
    signal head   : std_logic_vector(16 downto 0);

    -- Contadores del lado de entrada
    signal px_cnt : unsigned(7 downto 0) := (others => '0');
    signal ln_cnt : unsigned(7 downto 0) := (others => '0');

    -- Ultimo paquete de cada sensor
    signal sht_buf  : std_logic_vector(47 downto 0) := (others => '0');
    signal adxl_buf : std_logic_vector(47 downto 0) := (others => '0');
    signal ms_buf   : std_logic_vector(47 downto 0) := (others => '0');
    signal cal_buf  : std_logic_vector(95 downto 0) := (others => '0');
    signal p_sht, p_adxl, p_ms, p_cal : std_logic := '0';
    signal cal_have : std_logic := '0';
    signal cal_tmr  : integer range 0 to CAL_CLKS - 1 := 0;

    -- Secuenciador de salida
    type m_t   is (M_IDLE, M_HDR, M_LINE, M_SENS);
    type sel_t is (C_SHT, C_ADXL, C_MS, C_CAL);
    signal m_st    : m_t := M_IDLE;
    signal cur     : sel_t := C_SHT;
    signal hidx    : integer range 0 to 3 := 0;
    signal hdr_idx : std_logic_vector(7 downto 0) := (others => '0');
    signal px_tx   : integer range 0 to 255 := 0;
    signal sidx    : integer range 0 to 16 := 0;
    signal chk     : std_logic_vector(7 downto 0) := (others => '0');

    signal pad      : std_logic;
    signal n_data   : integer range 0 to 12;
    signal s_type   : std_logic_vector(7 downto 0);
    signal s_data   : std_logic_vector(7 downto 0);
    signal s_byte   : std_logic_vector(7 downto 0);
    signal h_byte   : std_logic_vector(7 downto 0);
    signal tx_valid_i : std_logic;
    signal tx_data_i  : std_logic_vector(7 downto 0);

    -- Byte k (0 = el de mas a la derecha) de un vector; 0 si k esta fuera de rango
    -- Se usa desplazamiento en vez de un corte con indice variable (Quartus no lo acepta)
    function pick(v : std_logic_vector; k : integer) return std_logic_vector is
        variable u : unsigned(v'length - 1 downto 0);
    begin
        if k >= 0 and 8 * k + 7 <= v'length - 1 then
            u := shift_right(unsigned(v), 8 * k);
            return std_logic_vector(u(7 downto 0));
        else
            return x"00";
        end if;
    end function;

begin

    head <= fifo(to_integer(rd_ptr));

    -- Relleno con ceros si empezo una linea nueva (frame_start) antes de completar la actual
    pad <= '1' when head(16) = '1' and px_tx /= 0 else '0';

    n_data <= 12 when cur = C_CAL else 6;

    s_type <= x"B1" when cur = C_SHT  else
              x"B2" when cur = C_ADXL else
              x"B3" when cur = C_MS   else
              x"B4";

    s_data <= pick(sht_buf,  sidx - 3) when cur = C_SHT  else
              pick(adxl_buf, sidx - 3) when cur = C_ADXL else
              pick(ms_buf,   sidx - 3) when cur = C_MS   else
              pick(cal_buf,  sidx - 3);

    s_byte <= x"A5"  when sidx = 0 else
              x"5A"  when sidx = 1 else
              s_type when sidx = 2 else
              chk    when sidx = n_data + 3 else
              s_data;

    h_byte <= x"A5"  when hidx = 0 else
              x"5A"  when hidx = 1 else
              x"A1"  when hidx = 2 else
              hdr_idx;

    tx_valid_i <= '1' when m_st = M_HDR or m_st = M_SENS or (m_st = M_LINE and cnt > 0) else '0';

    tx_data_i <= h_byte           when m_st = M_HDR else
                 x"00"            when m_st = M_LINE and pad = '1' else
                 head(7 downto 0) when m_st = M_LINE else
                 s_byte;

    tx_valid <= tx_valid_i;
    tx_data  <= tx_data_i;

    process(clk, rst_n)
        variable n    : integer range 0 to FIFO_DEPTH;
        variable push : boolean;
        variable pop  : boolean;
        variable xfer : boolean;
        variable fst  : std_logic;
    begin
        if rst_n = '0' then
            wr_ptr     <= (others => '0');
            rd_ptr     <= (others => '0');
            cnt        <= 0;
            px_cnt     <= (others => '0');
            ln_cnt     <= (others => '0');
            sht_buf    <= (others => '0');
            adxl_buf   <= (others => '0');
            ms_buf     <= (others => '0');
            cal_buf    <= (others => '0');
            p_sht      <= '0';
            p_adxl     <= '0';
            p_ms       <= '0';
            p_cal      <= '0';
            cal_have   <= '0';
            cal_tmr    <= 0;
            m_st       <= M_IDLE;
            cur        <= C_SHT;
            hidx       <= 0;
            hdr_idx    <= (others => '0');
            px_tx      <= 0;
            sidx       <= 0;
            chk        <= (others => '0');
            descartado <= '0';

        elsif rising_edge(clk) then

            push := false;
            pop  := false;
            n    := cnt;
            xfer := (tx_valid_i = '1' and tx_ready = '1');

            -------------------------------------------------------------------
            -- Entrada de pixeles: se etiquetan con indice de linea y marca de inicio
            -------------------------------------------------------------------
            if frame_start = '1' then
                px_cnt <= (others => '0');
                ln_cnt <= (others => '0');
            elsif pixel_valid = '1' then
                push   := true;
                px_cnt <= px_cnt + 1;
                if px_cnt = 255 then
                    ln_cnt <= ln_cnt + 1;
                end if;
            end if;

            -------------------------------------------------------------------
            -- Captura de paquetes de sensor (no se toca el que se esta enviando)
            -------------------------------------------------------------------
            if cal_have = '1' then                       -- reenvio periodico de la calibracion
                if cal_tmr = CAL_CLKS - 1 then
                    cal_tmr <= 0;
                    p_cal   <= '1';
                else
                    cal_tmr <= cal_tmr + 1;
                end if;
            end if;

            if sht_valid = '1' and not (m_st = M_SENS and cur = C_SHT) then
                sht_buf <= sht_data(47 downto 0);
                p_sht   <= '1';
            end if;

            if adxl_valid = '1' and not (m_st = M_SENS and cur = C_ADXL) then
                adxl_buf <= adxl_data(47 downto 0);
                p_adxl   <= '1';
            end if;

            if ms_valid = '1' then
                if ms_kind = '1' then
                    if not (m_st = M_SENS and cur = C_CAL) then
                        cal_buf  <= ms_data(95 downto 0);
                        cal_have <= '1';
                        p_cal    <= '1';
                        cal_tmr  <= 0;
                    end if;
                else
                    if not (m_st = M_SENS and cur = C_MS) then
                        ms_buf <= ms_data(47 downto 0);
                        p_ms   <= '1';
                    end if;
                end if;
            end if;

            -------------------------------------------------------------------
            -- Secuenciador de salida
            -------------------------------------------------------------------
            case m_st is

                when M_IDLE =>                           -- entre lineas: primero sensores
                    if p_cal = '1' then
                        cur <= C_CAL;  p_cal  <= '0'; sidx <= 0; chk <= (others => '0'); m_st <= M_SENS;
                    elsif p_ms = '1' then
                        cur <= C_MS;   p_ms   <= '0'; sidx <= 0; chk <= (others => '0'); m_st <= M_SENS;
                    elsif p_sht = '1' then
                        cur <= C_SHT;  p_sht  <= '0'; sidx <= 0; chk <= (others => '0'); m_st <= M_SENS;
                    elsif p_adxl = '1' then
                        cur <= C_ADXL; p_adxl <= '0'; sidx <= 0; chk <= (others => '0'); m_st <= M_SENS;
                    elsif n > 0 then
                        if head(16) = '1' then           -- hay un inicio de linea esperando
                            hdr_idx <= head(15 downto 8);
                            hidx    <= 0;
                            m_st    <= M_HDR;
                        else
                            pop := true;                 -- pixel sin cabecera (se perdio el inicio): descartar
                        end if;
                    end if;

                when M_HDR =>
                    if xfer then
                        if hidx = 3 then
                            px_tx <= 0;
                            m_st  <= M_LINE;
                        else
                            hidx <= hidx + 1;
                        end if;
                    end if;

                when M_LINE =>
                    if xfer then
                        if pad = '0' then
                            pop := true;
                        end if;
                        if px_tx = 255 then
                            m_st <= M_IDLE;
                        else
                            px_tx <= px_tx + 1;
                        end if;
                    end if;

                when M_SENS =>
                    if xfer then
                        if sidx >= 2 and sidx <= n_data + 2 then
                            chk <= chk xor s_byte;       -- tipo y bytes de datos
                        end if;
                        if sidx = n_data + 3 then
                            m_st <= M_IDLE;
                        else
                            sidx <= sidx + 1;
                        end if;
                    end if;

            end case;

            -------------------------------------------------------------------
            -- FIFO de pixeles
            -------------------------------------------------------------------
            if pop then
                rd_ptr <= rd_ptr + 1;
                n      := n - 1;
            end if;

            if push then
                if n = FIFO_DEPTH then
                    descartado <= '1';
                else
                    if px_cnt = 0 then fst := '1'; else fst := '0'; end if;
                    fifo(to_integer(wr_ptr)) <= fst & std_logic_vector(ln_cnt) & pixel_in;
                    wr_ptr <= wr_ptr + 1;
                    n      := n + 1;
                end if;
            end if;

            cnt <= n;

        end if;
    end process;

end architecture;