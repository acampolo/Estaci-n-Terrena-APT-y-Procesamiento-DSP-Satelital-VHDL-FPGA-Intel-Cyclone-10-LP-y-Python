library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Sincronizador y decimador de la senal de audio demodulada (salida de hilbert, 8 bits, 48 kHz).
-- Cada linea mide 6000 muestras:  sincronismo 400 (blanco) | porche 240 (negro) |
-- 256 pixeles x 20 muestras | porche 240 (negro).  Antes de la imagen hay una cabecera
-- de portadora continua (blanco) de cualquier duracion mayor que HDR_MIN muestras.
--
-- Diferencias con la version anterior:
--  * El sincronismo no se busca con un umbral fijo. Se mide el CONTRASTE entre las 400 muestras
--    del sincronismo y los dos porches negros que lo rodean (promedios de cientos de muestras),
--    asi que el ruido casi no afecta y no depende de la ganancia (PIXEL_SHIFT).
--  * Con el sincronismo enganchado, la siguiente linea se espera 6000 muestras despues, dentro de
--    una ventana de +-GATE. Si no aparece (ruido, el receptor se calla) se sigue "a ciegas" con el
--    mismo ritmo y solo se vuelve a buscar tras MAX_FALLOS lineas seguidas sin sincronismo.
--  * Cada linea se normaliza con su propio blanco (sincronismo) y negro (porches): el contraste
--    de la imagen no depende del nivel de entrada.  Cada pixel promedia las 16 muestras centrales.
--  * Los contadores no se desbordan: la cabecera puede durar 1 s o 15 s.
--  * frame_start sale una sola vez, justo antes del primer pixel de la primera linea que sigue a
--    la cabecera.  Tras N_LINEAS lineas no se entrega nada mas hasta la siguiente cabecera.
--
-- Salidas extra (se pueden dejar sin conectar):
--    locked : '1' mientras esta enganchado al sincronismo (sirve para un LED)
--    nivel  : nivel medio (0..255) del ultimo sincronismo; sirve para ajustar PIXEL_SHIFT
--
-- Requisito: valid_in separados al menos 16 ciclos de clk (con 48 kHz y 50 MHz hay ~1000).
entity sincronizador is
   generic (
        HDR_MIN    : positive := 12000;   
        MIN_NIVEL  : positive := 8;       -- RESTAURADO
        MAX_FALLOS : positive := 6;       
        N_LINEAS   : positive := 256      
    );
    port (
        clk         : in  std_logic;
        rst_n       : in  std_logic;

        data_in     : in  std_logic_vector(7 downto 0);
        valid_in    : in  std_logic;

        pixel_out   : out std_logic_vector(7 downto 0);
        pixel_valid : out std_logic;
        frame_start : out std_logic;

        locked      : out std_logic;
        nivel       : out std_logic_vector(7 downto 0)
    );
end entity;

architecture rtl of sincronizador is

    constant SYNC_N    : integer := 400;
    constant PORCH_N   : integer := 240;
    constant PIX_N     : integer := 20;
    constant LINE_N    : integer := 6000;
    constant GATE      : integer := 160;     -- tolerancia (+- muestras) al esperar el sincronismo
    constant LOOKAHEAD : integer := 96;      -- muestras que se espera tras el mejor punto antes de aceptarlo

    -- Suma acumulada de las ultimas 1024 muestras (dos copias: detector y generador de pixeles)
    type cs_t is array (0 to 1023) of unsigned(19 downto 0);
    signal ram1, ram2 : cs_t := (others => (others => '0'));
    signal wr_we      : std_logic := '0';
    signal wr_addr    : unsigned(9 downto 0) := (others => '0');
    signal wr_data    : unsigned(19 downto 0) := (others => '0');
    signal rd1_addr   : unsigned(9 downto 0) := (others => '0');
    signal rd2_addr   : unsigned(9 downto 0) := (others => '0');
    signal rd1_q      : unsigned(19 downto 0) := (others => '0');
    signal rd2_q      : unsigned(19 downto 0) := (others => '0');

    -- Detector de sincronismo
    type e_t is (E_IDLE, E_A1, E_A2, E_A3, E_B1, E_B2, E_C);
    signal es       : e_t := E_IDLE;
    signal n        : unsigned(19 downto 0) := (others => '0');   -- muestras recibidas (da la vuelta)
    signal cs_cur   : unsigned(19 downto 0) := (others => '0');
    signal warm     : integer range 0 to 1023 := 0;
    signal c240     : unsigned(19 downto 0) := (others => '0');
    signal c640     : unsigned(19 downto 0) := (others => '0');
    signal c880     : unsigned(19 downto 0) := (others => '0');

    signal has_best : std_logic := '0';
    signal best_d   : unsigned(20 downto 0) := (others => '0');
    signal best_n   : unsigned(19 downto 0) := (others => '0');
    signal best_s   : unsigned(17 downto 0) := (others => '0');
    signal best_b   : unsigned(17 downto 0) := (others => '0');
    signal last_s   : unsigned(17 downto 0) := (others => '0');
    signal last_b   : unsigned(17 downto 0) := (others => '0');
    signal lock     : std_logic := '0';
    signal exp_n    : unsigned(19 downto 0) := (others => '0');   -- donde se espera la siguiente linea
    signal fallos   : integer range 0 to 15 := 0;

    -- Nivel maximo y cabecera
    signal peak16   : unsigned(15 downto 0) := (others => '0');
    signal dec_cnt  : integer range 0 to 63 := 0;
    signal hi_run   : integer range 0 to 32767 := 0;
    signal lo_gap   : integer range 0 to 63 := 0;
    signal hdr_seen : std_logic := '0';

    -- Lineas entregadas
    signal emit_en    : std_logic := '1';
    signal lines_left : integer range 0 to 1023 := N_LINEAS;

    -- Peticion al generador de pixeles
    signal req      : std_logic := '0';
    signal req_v0   : unsigned(19 downto 0) := (others => '0');
    signal req_s    : unsigned(17 downto 0) := (others => '0');
    signal req_b    : unsigned(17 downto 0) := (others => '0');
    signal req_fs   : std_logic := '0';

    -- Generador de pixeles
    type g_t is (G_IDLE, G_DIV, G_GAIN, G_WAIT, G_R1, G_R2, G_R3, G_R4, G_MUL, G_OUT);
    signal gs       : g_t := G_IDLE;
    signal next_end : unsigned(19 downto 0) := (others => '0');
    signal pj       : integer range 0 to 255 := 0;
    signal dv_q     : unsigned(19 downto 0) := (others => '0');
    signal dv_rem   : unsigned(13 downto 0) := (others => '0');
    signal dv_den   : unsigned(13 downto 0) := (others => '0');
    signal dv_cnt   : integer range 0 to 19 := 0;
    signal ganancia   : integer range 0 to 16383 := 0;
    signal b_ref    : integer range 0 to 8191 := 0;
    signal ref_ok   : std_logic := '0';
    signal c_a      : unsigned(19 downto 0) := (others => '0');
    signal c_b      : unsigned(19 downto 0) := (others => '0');
    signal pix_r    : integer range 0 to 255 := 0;

begin

    locked <= lock;

    ---------------------------------------------------------------------------
    -- RAM de sumas acumuladas (sin reset, para que se infiera como memoria)
    ---------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if wr_we = '1' then
                ram1(to_integer(wr_addr)) <= wr_data;
                ram2(to_integer(wr_addr)) <= wr_data;
            end if;
            rd1_q <= ram1(to_integer(rd1_addr));
            rd2_q <= ram2(to_integer(rd2_addr));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Detector: acumula cada muestra y mide el contraste del sincronismo
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable x        : unsigned(7 downto 0);
        variable x16      : unsigned(15 downto 0);
        variable thr      : unsigned(7 downto 0);
        variable new_cs   : unsigned(19 downto 0);
        variable n_new    : unsigned(19 downto 0);
        variable tmp      : unsigned(19 downto 0);
        variable sw, p1, p2 : unsigned(17 downto 0);
        variable a6, b5, bs, b15 : unsigned(20 downto 0);
		  variable bk               : unsigned(20 downto 0);
        variable d        : unsigned(20 downto 0);
        variable dn       : signed(19 downto 0);
        variable cand, gate_ok, upd, decl, miss, do_line, new_frame : boolean;
        variable ln_n     : unsigned(19 downto 0);
        variable ln_s, ln_b : unsigned(17 downto 0);
        variable left     : integer;
        variable lvl      : unsigned(23 downto 0);
    begin
        if rst_n = '0' then
            es         <= E_IDLE;
            n          <= (others => '0');
            cs_cur     <= (others => '0');
            warm       <= 0;
            wr_we      <= '0';
            wr_addr    <= (others => '0');
            wr_data    <= (others => '0');
            rd1_addr   <= (others => '0');
            c240       <= (others => '0');
            c640       <= (others => '0');
            c880       <= (others => '0');
            has_best   <= '0';
            best_d     <= (others => '0');
            best_n     <= (others => '0');
            best_s     <= (others => '0');
            best_b     <= (others => '0');
            last_s     <= (others => '0');
            last_b     <= (others => '0');
            lock       <= '0';
            exp_n      <= (others => '0');
            fallos     <= 0;
            peak16     <= (others => '0');
            dec_cnt    <= 0;
            hi_run     <= 0;
            lo_gap     <= 0;
            hdr_seen   <= '0';
            emit_en    <= '1';
            lines_left <= N_LINEAS;
            req        <= '0';
            req_v0     <= (others => '0');
            req_s      <= (others => '0');
            req_b      <= (others => '0');
            req_fs     <= '0';
            nivel      <= (others => '0');

        elsif rising_edge(clk) then
            wr_we <= '0';
            req   <= '0';

            -- (solo simulacion) una muestra llego antes de terminar la anterior
            assert not (valid_in = '1' and es /= E_IDLE)
                report "sincronizador: muestra perdida, valid_in demasiado rapido" severity error;

            case es is

                when E_IDLE =>
                    if valid_in = '1' then
                        x      := unsigned(data_in);
                        new_cs := cs_cur + x;
                        n_new  := n + 1;
                        cs_cur  <= new_cs;
                        n       <= n_new;
                        wr_we   <= '1';
                        wr_addr <= n_new(9 downto 0);
                        wr_data <= new_cs;
                        if warm < 1023 then warm <= warm + 1; end if;

                        -- nivel maximo con caida lenta (para saber que es "alto" en la cabecera)
                        x16 := shift_left(resize(x, 16), 8);
                        if x16 > peak16 then
                            peak16  <= x16;
                            dec_cnt <= 0;
                        elsif dec_cnt = 63 then
                            dec_cnt <= 0;
                            peak16  <= peak16 - shift_right(peak16, 10);
                        else
                            dec_cnt <= dec_cnt + 1;
                        end if;

                        -- cabecera: portadora continua (admite caidas cortas)
                        thr := shift_right(peak16(15 downto 8), 1);
                        if thr < to_unsigned(MIN_NIVEL, 8) then
                            thr := to_unsigned(MIN_NIVEL, 8);
                        end if;
                        if x >= thr then
                            lo_gap <= 0;
                            if hi_run < 32767 then hi_run <= hi_run + 1; end if;
                        else
                            if lo_gap = 24 then hi_run <= 0; end if;
                            if lo_gap < 63 then lo_gap <= lo_gap + 1; end if;
                        end if;
                        if hi_run >= HDR_MIN then hdr_seen <= '1'; end if;

                        es <= E_A1;
                    end if;

                -- leer tres sumas acumuladas anteriores de la RAM (una lectura por ciclo)
                when E_A1 =>
                    tmp := n - to_unsigned(PORCH_N, 20);
                    rd1_addr <= tmp(9 downto 0);
                    es <= E_A2;

                when E_A2 =>
                    tmp := n - to_unsigned(PORCH_N + SYNC_N, 20);
                    rd1_addr <= tmp(9 downto 0);
                    es <= E_A3;

                when E_A3 =>
                    c240 <= rd1_q;
                    tmp := n - to_unsigned(2 * PORCH_N + SYNC_N, 20);
                    rd1_addr <= tmp(9 downto 0);
                    es <= E_B1;

                when E_B1 =>
                    c640 <= rd1_q;
                    es <= E_B2;

                when E_B2 =>
                    c880 <= rd1_q;
                    es <= E_C;

                when E_C =>
                    
                    sw := resize(c240 - c640, 18);
                    p2 := resize(cs_cur - c240, 18);
                    p1 := resize(c640 - c880, 18);
                    
                    bs := resize(p1, 21) + resize(p2, 21);
                    a6 := shift_left(resize(sw, 21), 2) + shift_left(resize(sw, 21), 1);   -- 6*sw
                    b5 := shift_left(bs, 2) + bs;                                          -- 5*bs
                    
                    -- NUEVA LÓGICA DE UMBRAL DINÁMICO
                    if lock = '1' then
                        bk := b5 + shift_right(b5, 2);   -- Enganchado: exige 1.25x
                    else
                        bk := shift_left(b5, 1);         -- Buscando: exige 2x
                    end if;
                    
                    cand := warm >= 900 and sw >= to_unsigned(SYNC_N * MIN_NIVEL, 18) and a6 >= bk;
                    if a6 >= b5 then
                        d := a6 - b5;
                    else
                        d := (others => '0');
                    end if;

                    dn      := signed(n - exp_n);
                    gate_ok := (dn >= -GATE) and (dn <= GATE);
                    if lock = '1' and not gate_ok then
                        cand := false;                      -- enganchado: solo se acepta cerca de lo esperado
                    end if;

                    upd := cand and (has_best = '0' or d > best_d);
                    if upd then
                        has_best <= '1';
                        best_d   <= d;
                        best_n   <= n;
                        best_s   <= sw;
                        best_b   <= resize(p1 + p2, 18);
                    end if;

                    -- aceptar el mejor punto cuando pasaron LOOKAHEAD muestras sin mejorar
                    decl := (not upd) and has_best = '1' and (n - best_n) = to_unsigned(LOOKAHEAD, 20);
                    -- enganchado y no llego nada en toda la ventana: se sigue con el ritmo anterior
                    miss := (not decl) and lock = '1' and has_best = '0'
                            and dn = to_signed(GATE + LOOKAHEAD + 2, 20);

                    do_line := false;
                    if decl then
                        has_best <= '0';
                        ln_n := best_n;  ln_s := best_s;  ln_b := best_b;
                        last_s <= best_s;  last_b <= best_b;
                        lvl := resize(resize(best_s, 24) * 41, 24);     -- sw/400 ~ sw*41/16384
                        nivel <= std_logic_vector(resize(shift_right(lvl, 14), 8));
                        lock   <= '1';
                        fallos <= 0;
                        exp_n  <= best_n + to_unsigned(LINE_N, 20);
                        do_line := true;
                    elsif miss then
                        ln_n := exp_n;  ln_s := last_s;  ln_b := last_b;
                        exp_n <= exp_n + to_unsigned(LINE_N, 20);
                        if fallos + 1 >= MAX_FALLOS then
                            lock     <= '0';
                            fallos   <= 0;
                            has_best <= '0';
                        else
                            fallos <= fallos + 1;
                        end if;
                        do_line := true;
                    end if;

                    -- entregar la linea a menos que ya se hayan entregado N_LINEAS desde la cabecera
                    if do_line then
                        new_frame := decl and hdr_seen = '1';
                        left := lines_left;
                        if new_frame then
                            hdr_seen <= '0';
                            left     := N_LINEAS;
                        end if;
                        if new_frame or emit_en = '1' then
                            req    <= '1';
                            req_v0 <= ln_n;
                            req_s  <= ln_s;
                            req_b  <= ln_b;
                            if new_frame then req_fs <= '1'; else req_fs <= '0'; end if;
                            
                            -- CORRECCIÓN DEL CONTADOR DE LÍNEAS
                            left := left - 1;
                            lines_left <= left;
                            if left <= 0 then 
                                emit_en <= '0'; 
                            else 
                                emit_en <= '1'; 
                            end if;
                        end if;
                    end if;

                    es <= E_IDLE;

            end case;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Generador de pixeles: promedia las 16 muestras centrales de cada grupo de 20,
    -- resta el negro de la linea y escala con el blanco de la linea
    ---------------------------------------------------------------------------
    process(clk, rst_n)
        variable w16, b16 : unsigned(27 downto 0);
        variable rn       : unsigned(13 downto 0);
        variable tmp      : unsigned(19 downto 0);
        variable dd       : unsigned(19 downto 0);
        variable s16      : integer;
        variable t_int    : integer;
        variable p_int    : integer;
        variable q        : integer;
    begin
        if rst_n = '0' then
            gs          <= G_IDLE;
            pixel_valid <= '0';
            frame_start <= '0';
            pixel_out   <= (others => '0');
            next_end    <= (others => '0');
            pj          <= 0;
            dv_q        <= (others => '0');
            dv_rem      <= (others => '0');
            dv_den      <= (others => '0');
            dv_cnt      <= 0;
            ganancia      <= 0;
            b_ref       <= 0;
            ref_ok      <= '0';
            rd2_addr    <= (others => '0');
            c_a         <= (others => '0');
            c_b         <= (others => '0');
            pix_r       <= 0;

        elsif rising_edge(clk) then
            pixel_valid <= '0';
            frame_start <= '0';

            case gs is

                when G_IDLE =>
                    if req = '1' then
                        -- referencias de 16 muestras: blanco = sync/25, negro = (porches)/30
                        w16 := shift_right(resize(resize(req_s, 28) * 41, 28), 10);
                        b16 := shift_right(resize(resize(req_b, 28) * 273, 28), 13);
                        b_ref <= to_integer(b16(12 downto 0));
                        if w16 > b16 + 16 then
                            ref_ok <= '1';
                            dv_den <= resize(w16(13 downto 0) - b16(13 downto 0), 14);
                        else
                            ref_ok <= '0';
                            dv_den <= to_unsigned(1, 14);
                        end if;
                        dv_q   <= to_unsigned(255 * 4096, 20);   -- ganancia = 255*4096 / (blanco - negro)
                        dv_rem <= (others => '0');
                        dv_cnt <= 0;
                        next_end <= req_v0 + to_unsigned(PIX_N, 20);
                        pj       <= 0;
                        if req_fs = '1' then frame_start <= '1'; end if;
                        gs <= G_DIV;
                    end if;

                when G_DIV =>                               -- division binaria, un bit por ciclo
                    rn := dv_rem(12 downto 0) & dv_q(19);
                    if rn >= dv_den then
                        dv_rem <= rn - dv_den;
                        dv_q   <= dv_q(18 downto 0) & '1';
                    else
                        dv_rem <= rn;
                        dv_q   <= dv_q(18 downto 0) & '0';
                    end if;
                    if dv_cnt = 19 then
                        gs <= G_GAIN;
                    else
                        dv_cnt <= dv_cnt + 1;
                    end if;

                when G_GAIN =>
                    if dv_q > to_unsigned(16383, 20) then
                        ganancia <= 16383;
                    else
                        ganancia <= to_integer(dv_q(13 downto 0));
                    end if;
                    gs <= G_WAIT;

                when G_WAIT =>                              -- esperar a que llegue el final de este pixel
                    dd := n - next_end;
                    if dd(19) = '0' then                    -- n >= next_end
                        if dd > to_unsigned(900, 20) then   -- ya se sobrescribieron esos datos: abandonar
                            gs <= G_IDLE;
                        else
                            gs <= G_R1;
                        end if;
                    end if;

                when G_R1 =>
                    tmp := next_end - to_unsigned(PIX_N - 2, 20);   -- inicio + 2
                    rd2_addr <= tmp(9 downto 0);
                    gs <= G_R2;

                when G_R2 =>
                    tmp := next_end - to_unsigned(2, 20);           -- fin - 2
                    rd2_addr <= tmp(9 downto 0);
                    gs <= G_R3;

                when G_R3 =>
                    c_a <= rd2_q;
                    gs <= G_R4;

                when G_R4 =>
                    c_b <= rd2_q;
                    gs <= G_MUL;

                when G_MUL =>
                    tmp   := c_b - c_a;
                    s16   := to_integer(tmp(12 downto 0));          -- suma de 16 muestras (<= 4080)
                    t_int := s16 - b_ref;
                    if ref_ok = '0' or t_int <= 0 then
                        p_int := 0;
                    else
                        q := (t_int * ganancia) / 4096;
                        if q > 255 then p_int := 255; else p_int := q; end if;
                    end if;
                    pix_r <= p_int;
                    gs <= G_OUT;

                when G_OUT =>
                    pixel_out   <= std_logic_vector(to_unsigned(pix_r, 8));
                    pixel_valid <= '1';
                    if pj = 255 then
                        gs <= G_IDLE;
                    else
                        pj       <= pj + 1;
                        next_end <= next_end + to_unsigned(PIX_N, 20);
                        gs       <= G_WAIT;
                    end if;

            end case;
        end if;
    end process;

end architecture;