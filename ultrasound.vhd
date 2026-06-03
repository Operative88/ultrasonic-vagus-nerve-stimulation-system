
--  Generator sygnalow sterujacych dla nieinwazyjnej ultradzwiekowej
--  stymulacji nerwu blednego (taVUS) na platforme Altera Cyclone IV
--  DE0-Nano (EP4CE22F17C6, zegar 50 MHz).
--
--  ARCHITEKTURA (4 bloki + top):
--    carrier_nco   - generator nosnej (akumulator fazy / DDS) - fala prostokatna
--    burst_gen     - paczki tonowe o czasie ON i czestotliwosci PRF
--    safety_timer  - twardy limit czasu ciaglej stymulacji
--    hbridge_drive - wyjscia komplementarne z czasem martwym (dead-time)
--    vns_top       - polaczenie calosci + mapowanie na piny DE0-Nano


-- 1) NOSNA: numerycznie sterowany oscylator (DDS / phase-accumulator)
--    Zaleta wzgledem zwyklego dzielnika: bardzo drobna rozdzielczosc czest.
--    f_out = PHASE_INC * f_clk / 2^ACC_W
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity carrier_nco is
  generic (
    ACC_W     : natural := 32;   -- szerokosc akumulatora fazy
    PHASE_INC : natural          -- przyrost fazy na takt (liczony w top)
  );
  port (
    clk : in  std_logic;
    rst : in  std_logic;         -- reset synchroniczny, aktywny '1'
    sq  : out std_logic          -- nosna jako fala prostokatna (50% wyp.)
  );
end entity;

architecture rtl of carrier_nco is
  signal acc : unsigned(ACC_W-1 downto 0) := (others => '0');
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        acc <= (others => '0');
      else
        acc <= acc + to_unsigned(PHASE_INC, ACC_W);
      end if;
    end if;
  end process;
  -- najstarszy bit akumulatora = fala prostokatna o czestotliwosci nosnej
  sq <= acc(ACC_W-1);
end architecture;


--------------------------------------------------------------------------------
-- 2) OBWIEDNIA: paczki tonowe (tone bursts)
--    PERIOD_CYCLES = f_clk / PRF        (okres powtarzania paczki)
--    ON_CYCLES     = czas ON [cykle]    (dlugosc pojedynczej paczki)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity burst_gen is
  generic (
    PERIOD_CYCLES : natural;
    ON_CYCLES     : natural
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    en    : in  std_logic;       -- '1' = generuj paczki
    burst : out std_logic        -- '1' w trakcie aktywnej paczki
  );
end entity;

architecture rtl of burst_gen is
  signal cnt : natural range 0 to PERIOD_CYCLES-1 := 0;
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' or en = '0' then
        cnt <= 0;
      elsif cnt = PERIOD_CYCLES-1 then
        cnt <= 0;
      else
        cnt <= cnt + 1;
      end if;
    end if;
  end process;

  burst <= '1' when (en = '1' and cnt < ON_CYCLES) else '0';
end architecture;


--------------------------------------------------------------------------------
-- 3) WATCHDOG BEZPIECZENSTWA: twardy limit czasu ciaglej stymulacji
--    Liczy sekundy gdy 'active'='1'. Po MAX_SEC zatrzaskuje stop (ok='0').
--    Odblokowanie tylko przez reset (rst). Zapobiega zbyt dlugiej ekspozycji.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity safety_timer is
  generic (
    CLK_HZ  : natural := 50_000_000;
    MAX_SEC : natural := 60
  );
  port (
    clk    : in  std_logic;
    rst    : in  std_logic;
    active : in  std_logic;      -- '1' gdy trwa stymulacja
    ok     : out std_logic       -- '1' = w limicie, '0' = przekroczono (latch)
  );
end entity;

architecture rtl of safety_timer is
  signal tick_cnt : natural range 0 to CLK_HZ-1 := 0;
  signal sec_cnt  : natural range 0 to MAX_SEC  := 0;
  signal tripped  : std_logic := '0';
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        tick_cnt <= 0;
        sec_cnt  <= 0;
        tripped  <= '0';
      elsif tripped = '0' and active = '1' then
        if tick_cnt = CLK_HZ-1 then
          tick_cnt <= 0;
          if sec_cnt = MAX_SEC-1 then
            tripped <= '1';      -- osiagnieto limit -> zatrzask
          else
            sec_cnt <= sec_cnt + 1;
          end if;
        else
          tick_cnt <= tick_cnt + 1;
        end if;
      end if;
    end if;
  end process;

  ok <= not tripped;
end architecture;


--------------------------------------------------------------------------------
-- 4) STEROWNIK MOSTKA H z czasem martwym (dead-time)
--    gate='0' -> oba wyjscia '0' (brak napiecia na przetworniku, brak DC).
--    gate='1' -> drv_p/drv_n komplementarne wg nosnej, z przerwa DT_CYCLES
--                przy kazdej zmianie, aby zapobiec przewodzeniu skrosnemu.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity hbridge_drive is
  generic (
    DT_CYCLES : natural := 5     -- czas martwy w taktach zegara
  );
  port (
    clk     : in  std_logic;
    rst     : in  std_logic;
    gate    : in  std_logic;     -- '1' = wyjscie aktywne
    carrier : in  std_logic;     -- nosna
    drv_p   : out std_logic;
    drv_n   : out std_logic
  );
end entity;

architecture rtl of hbridge_drive is
  signal prev_c : std_logic := '0';
  signal dt     : natural range 0 to DT_CYCLES := 0;
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' or gate = '0' then
        drv_p  <= '0';
        drv_n  <= '0';
        dt     <= 0;
        prev_c <= carrier;
      else
        if carrier /= prev_c then          -- zmiana nosnej -> wejdz w dead-time
          prev_c <= carrier;
          dt     <= DT_CYCLES;
          drv_p  <= '0';
          drv_n  <= '0';
        elsif dt /= 0 then                 -- trwa dead-time -> oba '0'
          dt    <= dt - 1;
          drv_p <= '0';
          drv_n <= '0';
        else                               -- normalna praca komplementarna
          drv_p <= carrier;
          drv_n <= not carrier;
        end if;
      end if;
    end if;
  end process;
end architecture;


--==============================================================================
-- 5) TOP-LEVEL dla DE0-Nano
--==============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vns_top is
  generic (
    F_CLK_HZ     : natural := 50_000_000;  -- zegar plytki
    F_CARRIER_HZ : natural := 40_000;      -- czest. nosnej (zob. uwagi!)
    BURST_ON_US  : natural := 300;         -- dlugosc paczki [us]
    PRF_HZ       : natural := 1_000;       -- czest. powtarzania paczek [Hz]
    MAX_ON_SEC   : natural := 60;          -- limit bezpieczenstwa [s]
    DEADTIME_NS  : natural := 100          -- czas martwy mostka H [ns]
  );
  port (
    CLOCK_50 : in  std_logic;
    KEY      : in  std_logic_vector(1 downto 0);  -- przyciski, AKTYWNE w '0'
    SW       : in  std_logic_vector(3 downto 0);  -- przelaczniki DIP
    LED      : out std_logic_vector(7 downto 0);
    -- wyjscia do ZEWNETRZNEGO stopnia mocy / mostka H:
    DRV_P    : out std_logic;   -- faza A
    DRV_N    : out std_logic;   -- faza B (komplementarna z dead-time)
    DRV_EN   : out std_logic    -- enable dla zewnetrznego sterownika mocy
  );
end entity;

architecture rtl of vns_top is

  -- ---- stale wyliczane z generykow (w czasie elaboracji) ----------------
  constant ACC_W         : natural := 32;
  -- przyrost fazy: round(2^ACC_W * f_carrier / f_clk); liczone w typie real,
  -- by uniknac przepelnienia integer przy mnozeniu 2^32.
  constant PHASE_INC     : natural :=
      natural( real(F_CARRIER_HZ) * (2.0 ** ACC_W) / real(F_CLK_HZ) );

  constant PERIOD_CYCLES : natural := F_CLK_HZ / PRF_HZ;
  -- (f_clk/1e6) liczone najpierw, by nie przepelnic integer:
  constant ON_CYCLES     : natural := (F_CLK_HZ / 1_000_000) * BURST_ON_US;
  constant DT_CYCLES     : natural := ((F_CLK_HZ / 1_000_000) * DEADTIME_NS) / 1000;

  -- ---- sygnaly wewnetrzne -----------------------------------------------
  signal rst       : std_logic;
  signal enable    : std_logic;   -- zyczenie uzytkownika (SW0)
  signal carrier   : std_logic;
  signal burst     : std_logic;
  signal safe_ok   : std_logic;
  signal gate      : std_logic;   -- wlasciwa aktywacja wyjscia
  signal drv_p_i   : std_logic;   -- wewnetrzne kopie wyjsc mostka
  signal drv_n_i   : std_logic;

  signal hb_cnt    : unsigned(25 downto 0) := (others => '0'); -- heartbeat LED
begin

  --------------------------------------------------------------------------
  -- Wejscia. Przyciski DE0-Nano sa aktywne w stanie niskim.
  --   KEY(0) -> reset (nacisniecie = reset, odblokowuje tez watchdog)
  --   SW(0)  -> wlacz/wylacz stymulacje
  --------------------------------------------------------------------------
  rst    <= not KEY(0);
  enable <= SW(0);

  -- bramkowanie: stymulacja aktywna tylko gdy uzytkownik wlaczyl,
  -- trwa paczka i watchdog nie zadzialal
  gate <= '1' when (enable = '1' and burst = '1' and safe_ok = '1') else '0';

  --------------------------------------------------------------------------
  -- Instancje blokow
  --------------------------------------------------------------------------
  u_nco : entity work.carrier_nco
    generic map ( ACC_W => ACC_W, PHASE_INC => PHASE_INC )
    port map    ( clk => CLOCK_50, rst => rst, sq => carrier );

  u_burst : entity work.burst_gen
    generic map ( PERIOD_CYCLES => PERIOD_CYCLES, ON_CYCLES => ON_CYCLES )
    port map    ( clk => CLOCK_50, rst => rst, en => enable, burst => burst );

  u_safe : entity work.safety_timer
    generic map ( CLK_HZ => F_CLK_HZ, MAX_SEC => MAX_ON_SEC )
    port map    ( clk => CLOCK_50, rst => rst,
                  active => enable, ok => safe_ok );

  u_drive : entity work.hbridge_drive
    generic map ( DT_CYCLES => DT_CYCLES )
    port map    ( clk => CLOCK_50, rst => rst, gate => gate,
                  carrier => carrier, drv_p => drv_p_i, drv_n => drv_n_i );

  DRV_P <= drv_p_i;
  DRV_N <= drv_n_i;

  -- enable zewnetrznego stopnia mocy musi OBEJMOWAC cale okno sterowania:
  -- zalaczony zanim pojawi sie napiecie i wciaz zalaczony az wyjscia opadna do '0'.
  DRV_EN <= gate or drv_p_i or drv_n_i;

  --------------------------------------------------------------------------
  -- Heartbeat ~1.5 Hz (oznaka, ze FPGA dziala)
  --------------------------------------------------------------------------
  process(CLOCK_50)
  begin
    if rising_edge(CLOCK_50) then
      hb_cnt <= hb_cnt + 1;
    end if;
  end process;

  --------------------------------------------------------------------------
  -- Sygnalizacja LED (aktywne w '1')
  --------------------------------------------------------------------------
  LED(0) <= enable;             -- zasilanie/wlaczone
  LED(1) <= gate;              -- trwa stymulacja (miga z PRF)
  LED(2) <= safe_ok;            -- '1' = w limicie czasu
  LED(3) <= not safe_ok;        -- '1' = ZADZIALAL watchdog (blad)
  LED(4) <= '0';
  LED(5) <= '0';
  LED(6) <= '0';
  LED(7) <= hb_cnt(25);         -- heartbeat

end architecture;