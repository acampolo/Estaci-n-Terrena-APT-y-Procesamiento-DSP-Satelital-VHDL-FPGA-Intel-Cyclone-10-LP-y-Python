library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity hilbert is
    generic (
        PIXEL_SHIFT : natural range 0 to 16 := 0     -- ganancia 2^PIXEL_SHIFT antes de tomar 8 bits
    );
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;
        data_in     : in  std_logic_vector(23 downto 0); -- Viene de tu I2S (puerto 'sal')
        data_valid  : in  std_logic;                     -- Viene de tu I2S (puerto 'flag')
        
        pixel_out   : out std_logic_vector(7 downto 0);  -- El nivel de gris final para UART/OLED
        pixel_valid : out std_logic                      -- Avisa que el píxel está listo
    );
end entity;

architecture rtl of hilbert is

    -- 1. Línea de retardo de 15 muestras
    type delay_array is array (0 to 14) of signed(23 downto 0);
    signal x : delay_array := (others => (others => '0'));

    -- 2. Solo necesitamos los 7 primeros coeficientes (por simetría)
    constant H0 : signed(15 downto 0) := to_signed( 1889, 16);
    constant H1 : signed(15 downto 0) := to_signed(  328, 16);
    constant H2 : signed(15 downto 0) := to_signed( 3336, 16);
    constant H3 : signed(15 downto 0) := to_signed(  496, 16);
    constant H4 : signed(15 downto 0) := to_signed( 6599, 16);
    constant H5 : signed(15 downto 0) := to_signed(  372, 16);
    constant H6 : signed(15 downto 0) := to_signed(20778, 16);

    -- Llenado inicial de la línea de retardo
    signal fill_cnt : unsigned(3 downto 0) := (others => '0');
    signal primed   : std_logic := '0';

    -- Pipeline de validez: 7 etapas
    signal valid_pipe : std_logic_vector(6 downto 0) := (others => '0');

    -- Etapa 1: Restas (un bit extra para evitar desbordamiento)
    signal d0, d1, d2, d3, d4, d5, d6 : signed(24 downto 0) := (others => '0');
    signal I_stg1 : signed(23 downto 0) := (others => '0');

    -- Etapa 2: Multiplicaciones (25 bits * 16 bits = 41 bits)
    signal p0, p1, p2, p3, p4, p5, p6 : signed(40 downto 0) := (others => '0');
    signal I_stg2 : signed(23 downto 0) := (others => '0');

    -- Etapa 3: Suma total, escalada por 2^15
    signal sum_Q : signed(43 downto 0) := (others => '0');
    signal I_stg3 : signed(23 downto 0) := (others => '0');

    -- Etapa 4: Valores absolutos con un bit extra de margen
    signal abs_I, abs_Q : unsigned(25 downto 0) := (others => '0');

    -- Etapa 5: Max y Min para la aproximación alfa-max beta-min
    signal max_val, min_val : unsigned(25 downto 0) := (others => '0');

    -- Etapa 6: Magnitud aproximada, Max + 3/8 Min
    signal mag_approx : unsigned(26 downto 0) := (others => '0');
	 
	 signal dc_acc : signed(39 downto 0) := (others => '0');
    signal media  : signed(23 downto 0) := (others => '0');
    signal x_in   : signed(23 downto 0) := (others => '0');

begin

    process(clk, rst_n)
        variable scaled : unsigned(26 downto 0);
    begin
        if rst_n = '0' then
            x <= (others => (others => '0'));
            fill_cnt <= (others => '0');
            primed <= '0';
            valid_pipe <= (others => '0');
            pixel_valid <= '0';
            pixel_out <= (others => '0');
            
        elsif rising_edge(clk) then
            
            -- Registro de desplazamiento de validez
            if data_valid = '1' and (primed = '1' or fill_cnt = 14) then
                valid_pipe(0) <= '1';
            else
                valid_pipe(0) <= '0';
            end if;
            valid_pipe(6 downto 1) <= valid_pipe(5 downto 0);
            pixel_valid <= valid_pipe(6);

            -- ETAPA 0: Ingreso del dato al Shift Register y DC Blocker
            if data_valid = '1' then
                -- Filtro IIR que emula la resta del x.mean() de Python
                dc_acc <= dc_acc + resize(signed(data_in), 40) - resize(media, 40);
                media  <= resize(shift_right(dc_acc, 12), 24);
                x_in   <= signed(data_in) - media;
                
                x(0) <= x_in; -- Entra la señal limpia sin DC Offset
                
                for i in 1 to 14 loop
                    x(i) <= x(i-1);
                end loop;

                if fill_cnt = 14 then
                    primed <= '1';
                else
                    fill_cnt <= fill_cnt + 1;
                end if;
             end if;

            -- ETAPA 1: Restas simétricas
            if valid_pipe(0) = '1' then
                d0 <= resize(x(0), 25) - resize(x(14), 25);
                d1 <= resize(x(1), 25) - resize(x(13), 25);
                d2 <= resize(x(2), 25) - resize(x(12), 25);
                d3 <= resize(x(3), 25) - resize(x(11), 25);
                d4 <= resize(x(4), 25) - resize(x(10), 25);
                d5 <= resize(x(5), 25) - resize(x(9),  25);
                d6 <= resize(x(6), 25) - resize(x(8),  25);
                I_stg1 <= x(7); -- El centro del filtro es la señal original, con retardo de grupo de 7 muestras
            end if;

            -- ETAPA 2: Multiplicaciones
            if valid_pipe(1) = '1' then
                p0 <= d0 * H0;
                p1 <= d1 * H1;
                p2 <= d2 * H2;
                p3 <= d3 * H3;
                p4 <= d4 * H4;
                p5 <= d5 * H5;
                p6 <= d6 * H6;
                I_stg2 <= I_stg1; -- Mantiene I alineado en el tiempo
            end if;

            -- ETAPA 3: Acumulador
            if valid_pipe(2) = '1' then
                sum_Q <= resize(p0, 44) + resize(p1, 44) + resize(p2, 44) +
                         resize(p3, 44) + resize(p4, 44) + resize(p5, 44) + resize(p6, 44);
                I_stg3 <= I_stg2;
            end if;

            -- ETAPA 4: Escalado y valor absoluto
            -- sum_Q está multiplicado por 2^15; tomamos desde el bit 15 hacia arriba
            -- con 26 bits para no perder el caso de señal cercana a plena escala
            if valid_pipe(3) = '1' then
                abs_Q <= unsigned(abs(sum_Q(40 downto 15)));
                abs_I <= unsigned(abs(resize(I_stg3, 26)));
            end if;

            -- ETAPA 5: Max y Min
            if valid_pipe(4) = '1' then
                if abs_I > abs_Q then
                    max_val <= abs_I; min_val <= abs_Q;
                else
                    max_val <= abs_Q; min_val <= abs_I;
                end if;
            end if;

            -- ETAPA 6: Magnitud ≈ Max + 3/8 Min, solo sumas y desplazamientos
            if valid_pipe(5) = '1' then
                mag_approx <= resize(max_val, 27) + 
                              shift_right(resize(min_val, 27), 2) + 
                              shift_right(resize(min_val, 27), 3);
            end if;

            -- ETAPA 7: Salida de 8 bits con ganancia 2^PIXEL_SHIFT y saturación
            -- con PIXEL_SHIFT = 0 equivale a tomar los bits 23 a 16
            if valid_pipe(6) = '1' then
                scaled := shift_right(mag_approx, 16 - PIXEL_SHIFT);
                if scaled > 255 then
                    pixel_out <= (others => '1');
                else
                    pixel_out <= std_logic_vector(scaled(7 downto 0));
                end if;
            end if;

        end if;
    end process;
end rtl;