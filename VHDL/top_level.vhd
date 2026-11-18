library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity top_level is
    generic (
        CLK_FREQ    : positive := 50_000_000;
        BAUD_RATE   : positive := 921600;
        PIXEL_SHIFT : natural range 0 to 16 := 4  -- Ganancia de contraste (0 = sin ganancia extra)
    );
    port (
        -- 1. Reloj maestro (50 MHz) y Reset (activo en bajo: '0' = reset)
        clk         : in    std_logic;
        rst_n       : in    std_logic;

        -- 2. Interfaz con el ADC PCM1808 (I2S)
        i2s_bck     : in    std_logic;
        i2s_lrc     : in    std_logic;
        i2s_data    : in    std_logic;

        -- 3. Controles opcionales de imagen (puedes conectarlos a switches o botones)
        dither_en   : in    std_logic := '1'; -- '1' = Dithering Bayer 4x4, '0' = Umbral fijo

        -- 4. Salida serie hacia la Laptop (USB-UART)
        uart_tx_pin : out   std_logic;

        -- 5. Bus I2C para la pantalla OLED SSD1306
        oled_scl    : inout std_logic;
        oled_sda    : inout std_logic;

        -- 6. Buses I2C independientes para cada sensor de la placa
        sht_scl     : inout std_logic;
        sht_sda     : inout std_logic;

        adxl_scl    : inout std_logic;
        adxl_sda    : inout std_logic;
        adxl_cs     : out   std_logic; -- Forzado a '1' para modo I2C
        adxl_sdo    : out   std_logic; -- Forzado a '0' para dirección 0x53

        ms_scl      : inout std_logic;
        ms_sda      : inout std_logic;
        ms_ps       : out   std_logic; -- Forzado a '1' para modo I2C
        ms_csb      : out   std_logic; -- Forzado a '1' para dirección 0x76

        -- 7. LEDs de diagnóstico en placa (1 = hubo error / desborde)
        led_err_oled  : out  std_logic;
        led_err_sht   : out  std_logic;
        led_err_adxl  : out  std_logic;
        led_err_ms    : out  std_logic;
        led_fifo_ovf  : out  std_logic;
        led_sync_lock : out  std_logic;

        -- 8. PINES DE DEPURACIÓN PARA EL ANALIZADOR LÓGICO (GPIO2)
        dbg_bck       : out  std_logic;
        dbg_lrc       : out  std_logic;
        dbg_data      : out  std_logic;
        dbg_valid     : out  std_logic
    );
end entity;

architecture structural of top_level is

    -- =========================================================================
    -- CABLES INTERNOS (Señales que conectan un módulo con otro)
    -- =========================================================================
    -- Cables entre I2S y Hilbert
    signal i2s_sample      : std_logic_vector(23 downto 0);
    signal i2s_valid       : std_logic;

    -- Cables entre Hilbert, OLED y Multiplexor de Paquetes
    signal hilbert_pixel   : std_logic_vector(7 downto 0);
    signal hilbert_valid   : std_logic;

    -- Cables de salida del sensor SHT31 (Temperatura y Humedad)
    signal sht_pkt_valid   : std_logic;
    signal sht_pkt_data    : std_logic_vector(127 downto 0);

    -- Cables de salida del sensor ADXL345 (Acelerómetro)
    signal adxl_pkt_valid  : std_logic;
    signal adxl_pkt_data   : std_logic_vector(127 downto 0);
	 
	 signal sync_pixel      : std_logic_vector(7 downto 0);
    signal sync_valid      : std_logic;
    signal sync_frame_strt : std_logic;

    -- Cables de salida del sensor MS5611 (Barómetro / Presión)
    signal ms_pkt_valid    : std_logic;
    signal ms_pkt_kind     : std_logic;
    signal ms_pkt_data     : std_logic_vector(127 downto 0);

    -- Cables entre el Multiplexor de Paquetes y el Transmisor UART
    signal mux_tx_data     : std_logic_vector(7 downto 0);
    signal mux_tx_valid    : std_logic;
    signal uart_tx_ready   : std_logic;

begin

	 dbg_bck   <= i2s_bck;
    dbg_lrc   <= i2s_lrc;
    dbg_data  <= i2s_data;
    dbg_valid <= hilbert_valid;
    -- =========================================================================
    -- CONFIGURACIÓN FÍSICA DE SENSORES (Pines fijos para activar modo I2C)
    -- =========================================================================
    adxl_cs  <= '1'; -- ADXL345 en modo I2C
    adxl_sdo <= '0'; -- ADXL345 en dirección 0x53
    ms_ps    <= '1'; -- MS5611 en modo I2C
    ms_csb   <= '1'; -- MS5611 en dirección 0x76 (CSB en alto)

    -- =========================================================================
    -- 1. INSTANCIA DEL RECEPTOR I2S
    -- =========================================================================
    u_i2s : entity work.i2s
        port map (
            clk  => clk,
            R    => rst_n,        -- Tu puerto 'R' es activo en bajo, va directo a rst_n
            BCK  => i2s_bck,
            LRC  => i2s_lrc,
            Data => i2s_data,
            sal  => i2s_sample,   -- Sale muestra de 24 bits
            flag => i2s_valid     -- Sale pulso de validación
        );

    -- =========================================================================
    -- 2. INSTANCIA DEL DEMODULADOR FIR DE HILBERT
    -- =========================================================================
    u_hilbert : entity work.hilbert
        generic map (
            PIXEL_SHIFT => PIXEL_SHIFT
        )
        port map (
            clk         => clk,
            rst_n       => rst_n,
            data_in     => i2s_sample,    -- Recibe los 24 bits del I2S
            data_valid  => i2s_valid,     -- Recibe el flag del I2S
            pixel_out   => hilbert_pixel, -- Entrega el píxel de 8 bits (0..255)
            pixel_valid => hilbert_valid  -- Avisa que el píxel está listo
        );
	
	-- =========================================================================
    -- 2.5 INSTANCIA DEL SINCRONIZADOR Y DECIMADOR APT (NUEVO)
    -- =========================================================================
    u_sincronizador : entity work.sincronizador
        port map (
            clk         => clk,
            rst_n       => rst_n,
            data_in     => hilbert_pixel,   -- Entra la basura a 48kHz
            valid_in    => hilbert_valid,
            pixel_out   => sync_pixel,      -- Salen 256 píxeles puros
            pixel_valid => sync_valid,
            frame_start => sync_frame_strt,
            locked      => led_sync_lock,   -- <--- CONECTADO AL LED
            nivel       => open             -- <--- SE DEJA ABIERTO
        );
    -- =========================================================================
    -- 3. INSTANCIA DEL PROCESADOR Y DRIVER PARA PANTALLA OLED (128x64)
    --    Muestra la imagen mientras lleguen píxeles y las barras de sensores si no
    -- =========================================================================
    u_oled : entity work.oled_img
        generic map (
            CLK_FREQ       => CLK_FREQ,
            I2C_FREQ       => 400_000,
            OLED_ADDR      => "0111100",       -- Dirección 0x3C
            IMG_TIMEOUT_MS => 15000           -- Mantiene la imagen 15 s en la OLED al terminar
        )
        port map (
            clk         => clk,
            rst_n       => rst_n,
            pixel_in    => sync_pixel,      -- ANTES: hilbert_pixel
            pixel_valid => sync_valid,      -- ANTES: hilbert_valid
            frame_start => sync_frame_strt, -- ANTES: frame_start (pin físico)
            dither_en   => dither_en,
            thresh      => x"80",         -- Umbral fijo en 128 (si dither_en = '0')
            sht_valid   => sht_pkt_valid, -- Barras de temperatura y humedad
            sht_data    => sht_pkt_data,
            adxl_valid  => adxl_pkt_valid, -- Barras del acelerómetro
            adxl_data   => adxl_pkt_data,
            scl         => oled_scl,
            sda         => oled_sda,
            nack_err    => led_err_oled,
            modo_sensores => open
        );

    -- =========================================================================
    -- 4. INSTANCIAS DE LOS 3 SENSORES I2C
    -- =========================================================================

    -- Sensor 0: SHT31 (Temperatura y Humedad, dirección 0x44)
    u_sht31 : entity work.sensor_reader
        generic map (
            CLK_FREQ  => CLK_FREQ,
            I2C_FREQ  => 400_000,
            SENSOR    => 0,
            I2C_ADDR  => "1000100",       -- 0x44
            PERIOD_MS => 200              -- Lee cada 200 ms (5 Hz)
        )
        port map (
            clk       => clk,
            rst_n     => rst_n,
            pkt_valid => sht_pkt_valid,
            pkt_kind  => open,            -- SHT31 solo tiene paquetes de medida
            pkt_len   => open,            -- Longitud fija conocida por packet_mux
            pkt_data  => sht_pkt_data,
            scl       => sht_scl,
            sda       => sht_sda,
            nack_err  => led_err_sht
        );

    -- Sensor 1: ADXL345 (Acelerómetro 3 ejes, dirección 0x53)
    u_adxl345 : entity work.sensor_reader
        generic map (
            CLK_FREQ  => CLK_FREQ,
            I2C_FREQ  => 400_000,
            SENSOR    => 1,
            I2C_ADDR  => "1010011",       -- 0x53
            PERIOD_MS => 100              -- Lee cada 100 ms (10 Hz)
        )
        port map (
            clk       => clk,
            rst_n     => rst_n,
            pkt_valid => adxl_pkt_valid,
            pkt_kind  => open,
            pkt_len   => open,
            pkt_data  => adxl_pkt_data,
            scl       => adxl_scl,
            sda       => adxl_sda,
            nack_err  => led_err_adxl
        );

    -- Sensor 2: MS5611 (Barómetro y Temperatura, dirección 0x76)
    u_ms5611 : entity work.sensor_reader
        generic map (
            CLK_FREQ  => CLK_FREQ,
            I2C_FREQ  => 400_000,
            SENSOR    => 2,
            I2C_ADDR  => "1110110",       -- 0x76
            PERIOD_MS => 250              -- Lee cada 250 ms (4 Hz)
        )
        port map (
            clk       => clk,
            rst_n     => rst_n,
            pkt_valid => ms_pkt_valid,
            pkt_kind  => ms_pkt_kind,     -- Importante: distingue calibración ('1') de medida ('0')
            pkt_len   => open,
            pkt_data  => ms_pkt_data,
            scl       => ms_scl,
            sda       => ms_sda,
            nack_err  => led_err_ms
        );

    -- =========================================================================
    -- 5. INSTANCIA DEL MULTIPLEXOR DE TRAMAS (Imágenes + Telemetría)
    -- =========================================================================
    u_packet_mux : entity work.packet_mux
        generic map (
            CLK_FREQ     => CLK_FREQ,
            CAL_PERIOD_S => 10            -- Reenvía la calibración del MS5611 cada 10 s
        )
        port map (
            clk         => clk,
            rst_n       => rst_n,
            pixel_valid => sync_valid,      -- ANTES: hilbert_valid
            pixel_in    => sync_pixel,      -- ANTES: hilbert_pixel
            frame_start => sync_frame_strt, -- ANTES: frame_start (pin físico)
            sht_valid   => sht_pkt_valid,
            sht_data    => sht_pkt_data,
            adxl_valid  => adxl_pkt_valid,
            adxl_data   => adxl_pkt_data,
            ms_valid    => ms_pkt_valid,
            ms_kind     => ms_pkt_kind,
            ms_data     => ms_pkt_data,
            tx_data     => mux_tx_data,
            tx_valid    => mux_tx_valid,
            tx_ready    => uart_tx_ready,
            descartado  => led_fifo_ovf
        );

    -- =========================================================================
    -- 6. INSTANCIA DEL TRANSMISOR UART (921600 baudios, 8N1)
    -- =========================================================================
    u_uart_tx : entity work.uart_tx
        generic map (
            CLK_FREQ => CLK_FREQ,
            BAUD     => BAUD_RATE
        )
        port map (
            clk   => clk,
            rst_n => rst_n,
            data  => mux_tx_data,
            valid => mux_tx_valid,
            ready => uart_tx_ready,
            tx    => uart_tx_pin
        );

end architecture;