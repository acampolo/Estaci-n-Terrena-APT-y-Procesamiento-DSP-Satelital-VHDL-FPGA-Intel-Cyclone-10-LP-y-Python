library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Transmisor UART 8N1 generico: transmite bytes sueltos, sin ninguna logica de paquetes.
-- Interfaz valid/ready: el byte se acepta en el ciclo en que valid = '1' y ready = '1'.
-- ready = '1' en reposo y tambien en el ultimo ciclo del bit de parada, asi los bytes
-- salen pegados uno tras otro sin hueco extra entre ellos.
-- Con CLK_FREQ = 50 MHz y BAUD = 921600, BIT_TICKS = 54 (baud real 925926, error 0.5 %).
entity uart_tx is
    generic (
        CLK_FREQ : positive := 50_000_000;   -- Hz, reloj de la FPGA
        BAUD     : positive := 921600
    );
    port (
        clk   : in  std_logic;
        rst_n : in  std_logic;
        data  : in  std_logic_vector(7 downto 0);
        valid : in  std_logic;
        ready : out std_logic;
        tx    : out std_logic
    );
end entity;

architecture rtl of uart_tx is

    constant BIT_TICKS : positive := (CLK_FREQ + BAUD / 2) / BAUD;

    type state_t is (IDLE, START, DATA_ST, STOP);
    signal state    : state_t := IDLE;
    signal tick_cnt : natural range 0 to BIT_TICKS - 1 := 0;
    signal bit_idx  : natural range 0 to 7 := 0;
    signal shreg    : std_logic_vector(7 downto 0) := (others => '0');

begin

    ready <= '1' when state = IDLE or (state = STOP and tick_cnt = BIT_TICKS - 1) else '0';

    process(clk, rst_n)
    begin
        if rst_n = '0' then
            state    <= IDLE;
            tick_cnt <= 0;
            bit_idx  <= 0;
            shreg    <= (others => '0');
            tx       <= '1';

        elsif rising_edge(clk) then

            case state is

                when IDLE =>
                    tx <= '1';
                    if valid = '1' then
                        shreg    <= data;
                        tick_cnt <= 0;
                        tx       <= '0';            -- bit de inicio
                        state    <= START;
                    end if;

                when START =>
                    if tick_cnt = BIT_TICKS - 1 then
                        tick_cnt <= 0;
                        tx       <= shreg(0);       -- primer bit de datos, LSB primero
                        shreg    <= '1' & shreg(7 downto 1);
                        bit_idx  <= 0;
                        state    <= DATA_ST;
                    else
                        tick_cnt <= tick_cnt + 1;
                    end if;

                when DATA_ST =>
                    if tick_cnt = BIT_TICKS - 1 then
                        tick_cnt <= 0;
                        if bit_idx = 7 then
                            tx    <= '1';           -- bit de parada
                            state <= STOP;
                        else
                            tx      <= shreg(0);
                            shreg   <= '1' & shreg(7 downto 1);
                            bit_idx <= bit_idx + 1;
                        end if;
                    else
                        tick_cnt <= tick_cnt + 1;
                    end if;

                when STOP =>
                    if tick_cnt = BIT_TICKS - 1 then
                        tick_cnt <= 0;
                        if valid = '1' then         -- byte siguiente sin pasar por IDLE
                            shreg <= data;
                            tx    <= '0';
                            state <= START;
                        else
                            state <= IDLE;
                        end if;
                    else
                        tick_cnt <= tick_cnt + 1;
                    end if;

            end case;

        end if;
    end process;

end architecture;