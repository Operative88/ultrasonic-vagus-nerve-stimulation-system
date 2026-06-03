library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb is end entity;

architecture sim of tb is
  signal clk  : std_logic := '0';
  signal key  : std_logic_vector(1 downto 0) := "11"; -- nie wcisniete
  signal sw   : std_logic_vector(3 downto 0) := "0000";
  signal led  : std_logic_vector(7 downto 0);
  signal dp, dn, den : std_logic;

  -- liczniki obserwacyjne
  signal toggles_in_burst : integer := 0;
  signal out_high_when_off: integer := 0;
  signal prev_dp : std_logic := '0';

  -- osobny test watchdoga z malym CLK_HZ
  signal s_active : std_logic := '1';
  signal s_ok     : std_logic;
  signal s_rst    : std_logic := '1';
begin
  -- 50 MHz -> okres 20 ns
  clk <= not clk after 10 ns;

  uut : entity work.vns_top
    generic map (
      F_CLK_HZ => 50_000_000, F_CARRIER_HZ => 1_000_000,
      BURST_ON_US => 10, PRF_HZ => 10_000,
      MAX_ON_SEC => 60, DEADTIME_NS => 100 )
    port map ( CLOCK_50 => clk, KEY => key, SW => sw, LED => led,
               DRV_P => dp, DRV_N => dn, DRV_EN => den );

  -- maly watchdog: CLK_HZ=100, MAX_SEC=1 -> trip po 100 taktach
  wd : entity work.safety_timer
    generic map ( CLK_HZ => 100, MAX_SEC => 1 )
    port map ( clk => clk, rst => s_rst, active => s_active, ok => s_ok );

  -- obserwacja wyjscia mostka
  obs : process(clk)
  begin
    if rising_edge(clk) then
      if den = '1' and dp /= prev_dp then
        toggles_in_burst <= toggles_in_burst + 1;
      end if;
      if den = '0' and dp = '1' then
        out_high_when_off <= out_high_when_off + 1; -- powinno zostac 0
      end if;
      prev_dp <= dp;
    end if;
  end process;

  stim : process
  begin
    -- reset
    key <= "10"; wait for 100 ns; key <= "11";
    -- wlacz stymulacje
    sw(0) <= '1';
    wait for 250 us;   -- ~2.5 okresy PRF

    report "DRV_P przelaczen w trakcie paczek = " & integer'image(toggles_in_burst);
    report "DRV_P=1 gdy wyjscie wylaczone (ma byc 0) = " & integer'image(out_high_when_off);
    assert toggles_in_burst > 10
      report "BLAD: brak nosnej w trakcie paczki" severity failure;
    assert out_high_when_off = 0
      report "BLAD: napiecie na wyjsciu poza paczka" severity failure;

    -- test watchdoga
    s_rst <= '0';
    wait for 3 us;     -- > 100 taktow (100 * 20ns = 2us)
    assert s_ok = '0'
      report "BLAD: watchdog nie zatrzasnal stopu" severity failure;
    report "Watchdog zadzialal poprawnie (ok=0)";

    -- reset odblokowuje
    s_rst <= '1'; wait for 100 ns; s_rst <= '0'; wait for 100 ns;
    assert s_ok = '1'
      report "BLAD: reset nie odblokowal watchdoga" severity failure;
    report "Reset odblokowal watchdog (ok=1)";

    report "=== WSZYSTKIE TESTY OK ===" severity note;
    std.env.stop;
  end process;
end architecture;