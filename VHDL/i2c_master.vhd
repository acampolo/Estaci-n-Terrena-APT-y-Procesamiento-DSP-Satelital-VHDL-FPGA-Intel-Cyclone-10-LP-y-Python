library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Maestro I2C a nivel de byte. Cada comando hace, en este orden y según sus banderas:
--   START o START repetido, escribir o leer un byte, STOP
-- Se instancia una vez por bus. Soporta estiramiento de reloj del esclavo.
entity i2c_master is
    generic (
        CLK_FREQ : positive := 50_000_000;   -- Hz, reloj de la FPGA
        I2C_FREQ : positive := 400_000       -- Hz, frecuencia de SCL como máximo
    );
    port (
        clk       : in    std_logic;
        rst_n     : in    std_logic;

        cmd_valid : in    std_logic;                      -- pulso de un ciclo, solo se acepta si cmd_ready = '1'
        cmd_start : in    std_logic;                      -- generar START o START repetido antes del byte
        cmd_stop  : in    std_logic;                      -- generar STOP después del byte
        cmd_write : in    std_logic;                      -- escribir wr_data
        cmd_read  : in    std_logic;                      -- leer un byte
        cmd_ack   : in    std_logic;                      -- en lectura: '1' responde ACK, '0' responde NACK, usar '0' en el último byte
        wr_data   : in    std_logic_vector(7 downto 0);

        cmd_ready : out   std_logic;                      -- '1' cuando puede recibir un comando
        done      : out   std_logic;                      -- pulso de un ciclo al terminar el comando
        rd_data   : out   std_logic_vector(7 downto 0);   -- válido en done tras una lectura
        ack_err   : out   std_logic;                      -- válido en done tras una escritura: '1' si el esclavo no respondió ACK

        scl       : inout std_logic;                      -- drenador abierto, requiere pull-up externo
        sda       : inout std_logic
    );
end entity;

architecture rtl of i2c_master is

    -- Un cuarto del periodo de SCL, redondeado hacia arriba para no pasar de I2C_FREQ
    constant Q_TICKS : positive := (CLK_FREQ + 4 * I2C_FREQ - 1) / (4 * I2C_FREQ);

    type st_t is (S_IDLE, S_START, S_BITS, S_STOP, S_DONE);
    signal st       : st_t := S_IDLE;
    signal ph       : integer range 0 to 3 := 0;
    signal tmr      : integer range 0 to Q_TICKS - 1 := 0;
    signal tick     : std_logic;

    signal scl_low  : std_logic := '0';
    signal sda_low  : std_logic := '0';
    signal scl_in   : std_logic;
    signal sda_in   : std_logic;

    signal bit_cnt  : integer range 0 to 8 := 0;
    signal shreg    : std_logic_vector(7 downto 0) := (others => '0');
    signal f_start  : std_logic := '0';
    signal f_stop   : std_logic := '0';
    signal f_write  : std_logic := '0';
    signal f_read   : std_logic := '0';
    signal f_ack    : std_logic := '0';

begin

    scl <= '0' when scl_low = '1' else 'Z';
    sda <= '0' when sda_low = '1' else 'Z';
    scl_in <= to_x01(scl);
    sda_in <= to_x01(sda);

    cmd_ready <= '1' when st = S_IDLE else '0';
    tick      <= '1' when tmr = Q_TICKS - 1 else '0';

    -- Base de tiempo de un cuarto de periodo
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            tmr <= 0;
        elsif rising_edge(clk) then
            if tmr = Q_TICKS - 1 then
                tmr <= 0;
            else
                tmr <= tmr + 1;
            end if;
        end if;
    end process;

    process(clk, rst_n)
    begin
        if rst_n = '0' then
            st      <= S_IDLE;
            ph      <= 0;
            bit_cnt <= 0;
            scl_low <= '0';
            sda_low <= '0';
            done    <= '0';
            rd_data <= (others => '0');
            ack_err <= '0';
            shreg   <= (others => '0');
            f_start <= '0'; f_stop <= '0'; f_write <= '0'; f_read <= '0'; f_ack <= '0';

        elsif rising_edge(clk) then

            done <= '0';

            case st is

                when S_IDLE =>
                    if cmd_valid = '1' then
                        f_start <= cmd_start;
                        f_stop  <= cmd_stop;
                        f_write <= cmd_write;
                        f_read  <= cmd_read;
                        f_ack   <= cmd_ack;
                        shreg   <= wr_data;
                        ack_err <= '0';
                        ph      <= 0;
                        bit_cnt <= 0;
                        if cmd_start = '1' then
                            st <= S_START;
                        elsif cmd_write = '1' or cmd_read = '1' then
                            st <= S_BITS;
                        elsif cmd_stop = '1' then
                            st <= S_STOP;
                        else
                            st <= S_DONE;
                        end if;
                    end if;

                when S_START =>
                    if tick = '1' then
                        case ph is
                            when 0 =>                               -- SDA arriba con SCL abajo
                                sda_low <= '0';
                                ph <= 1;
                            when 1 =>                               -- soltar SCL
                                scl_low <= '0';
                                ph <= 2;
                            when 2 =>                               -- con SCL arriba, bajar SDA
                                if scl_in = '1' then
                                    sda_low <= '1';
                                    ph <= 3;
                                end if;
                            when others =>                          -- bajar SCL
                                scl_low <= '1';
                                ph <= 0;
                                if f_write = '1' or f_read = '1' then
                                    st <= S_BITS;
                                elsif f_stop = '1' then
                                    st <= S_STOP;
                                else
                                    st <= S_DONE;
                                end if;
                        end case;
                    end if;

                when S_BITS =>
                    if tick = '1' then
                        case ph is
                            when 0 =>                               -- preparar SDA con SCL abajo
                                if bit_cnt < 8 then
                                    if f_write = '1' then
                                        sda_low <= not shreg(7);
                                    else
                                        sda_low <= '0';
                                    end if;
                                else
                                    if f_write = '1' then
                                        sda_low <= '0';             -- el esclavo responde el ACK
                                    else
                                        sda_low <= f_ack;           -- el maestro responde ACK bajando SDA
                                    end if;
                                end if;
                                ph <= 1;
                            when 1 =>                               -- soltar SCL
                                scl_low <= '0';
                                ph <= 2;
                            when 2 =>                               -- con SCL arriba, muestrear
                                if scl_in = '1' then
                                    if bit_cnt < 8 then
                                        if f_write = '1' then
                                            shreg <= shreg(6 downto 0) & '0';
                                        else
                                            shreg <= shreg(6 downto 0) & sda_in;
                                        end if;
                                    elsif f_write = '1' then
                                        ack_err <= sda_in;          -- '1' significa NACK
                                    end if;
                                    ph <= 3;
                                end if;
                            when others =>                          -- bajar SCL y pasar al siguiente bit
                                scl_low <= '1';
                                ph <= 0;
                                if bit_cnt = 8 then
                                    bit_cnt <= 0;
                                    if f_read = '1' then
                                        rd_data <= shreg;
                                    end if;
                                    if f_stop = '1' then
                                        st <= S_STOP;
                                    else
                                        st <= S_DONE;
                                    end if;
                                else
                                    bit_cnt <= bit_cnt + 1;
                                end if;
                        end case;
                    end if;

                when S_STOP =>
                    if tick = '1' then
                        case ph is
                            when 0 =>                               -- SDA abajo con SCL abajo
                                sda_low <= '1';
                                ph <= 1;
                            when 1 =>                               -- soltar SCL
                                scl_low <= '0';
                                ph <= 2;
                            when 2 =>                               -- con SCL arriba, soltar SDA
                                if scl_in = '1' then
                                    sda_low <= '0';
                                    ph <= 3;
                                end if;
                            when others =>
                                ph <= 0;
                                st <= S_DONE;
                        end case;
                    end if;

                when S_DONE =>
                    done <= '1';
                    st   <= S_IDLE;

            end case;

        end if;
    end process;

end architecture;