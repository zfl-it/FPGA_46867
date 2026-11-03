-- Created by IP Generator (Version 2022.2-SP6.4 build 146967)
-- Instantiation Template
--
-- Insert the following codes into your VHDL file.
--   * Change the_instance_name to your own instance name.
--   * Change the net names in the port map.


COMPONENT PLL_50mHZ
  PORT (
    clkout0 : OUT STD_LOGIC;
    lock : OUT STD_LOGIC;
    clkin1 : IN STD_LOGIC;
    clkout0_n : OUT STD_LOGIC;
    clkout1 : OUT STD_LOGIC;
    clkout1_n : OUT STD_LOGIC;
    clkout2 : OUT STD_LOGIC;
    clkout2_n : OUT STD_LOGIC;
    clkout3 : OUT STD_LOGIC;
    clkout3_n : OUT STD_LOGIC;
    clkout4 : OUT STD_LOGIC;
    clkout1_ph90 : OUT STD_LOGIC;
    clkout1_ph180 : OUT STD_LOGIC;
    clkout1_ph270 : OUT STD_LOGIC;
    clksscg : OUT STD_LOGIC;
    clkout5 : OUT STD_LOGIC;
    clkout6 : OUT STD_LOGIC;
    clkoutphy : OUT STD_LOGIC;
    clkoutphy_n : OUT STD_LOGIC;
    clkoutf : OUT STD_LOGIC;
    clkoutf_n : OUT STD_LOGIC;
    dps_done : OUT STD_LOGIC;
    apb_ready : OUT STD_LOGIC;
    apb_rdata : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    clkin : IN STD_LOGIC;
    pll_pd : IN STD_LOGIC;
    clkoutpd : IN STD_LOGIC;
    clk4phasepd : IN STD_LOGIC;
    ssc_en_n : IN STD_LOGIC;
    fbdiv : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
    frac : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
    clkin1_p : IN STD_LOGIC;
    clkin1_n : IN STD_LOGIC;
    clkin2 : IN STD_LOGIC;
    clkin2_p : IN STD_LOGIC;
    clkin2_n : IN STD_LOGIC;
    clkfb : IN STD_LOGIC;
    clkin_sel : IN STD_LOGIC;
    pll_pwd : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    clkout0_syn : IN STD_LOGIC;
    clkout1_syn : IN STD_LOGIC;
    clkout2_syn : IN STD_LOGIC;
    clkout3_syn : IN STD_LOGIC;
    clkout4_syn : IN STD_LOGIC;
    clkout5_syn : IN STD_LOGIC;
    clkout6_syn : IN STD_LOGIC;
    clkoutphy_syn : IN STD_LOGIC;
    clkoutf_syn : IN STD_LOGIC;
    clkout0_bufce : IN STD_LOGIC;
    clkout1_bufce : IN STD_LOGIC;
    clkout2_bufce : IN STD_LOGIC;
    clkout3_bufce : IN STD_LOGIC;
    clkout4_bufce : IN STD_LOGIC;
    clkout5_bufce : IN STD_LOGIC;
    clkout6_bufce : IN STD_LOGIC;
    clkoutf_bufce : IN STD_LOGIC;
    dps_clk : IN STD_LOGIC;
    dps_en : IN STD_LOGIC;
    dps_dir : IN STD_LOGIC;
    apb_clk : IN STD_LOGIC;
    apb_rst_n : IN STD_LOGIC;
    apb_addr : IN STD_LOGIC_VECTOR(4 DOWNTO 0);
    apb_sel : IN STD_LOGIC;
    apb_en : IN STD_LOGIC;
    apb_write : IN STD_LOGIC;
    apb_wdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0)
  );
END COMPONENT;


the_instance_name : PLL_50mHZ
  PORT MAP (
    clkout0 => clkout0,
    lock => lock,
    clkin1 => clkin1,
    clkout0_n => clkout0_n,
    clkout1 => clkout1,
    clkout1_n => clkout1_n,
    clkout2 => clkout2,
    clkout2_n => clkout2_n,
    clkout3 => clkout3,
    clkout3_n => clkout3_n,
    clkout4 => clkout4,
    clkout1_ph90 => clkout1_ph90,
    clkout1_ph180 => clkout1_ph180,
    clkout1_ph270 => clkout1_ph270,
    clksscg => clksscg,
    clkout5 => clkout5,
    clkout6 => clkout6,
    clkoutphy => clkoutphy,
    clkoutphy_n => clkoutphy_n,
    clkoutf => clkoutf,
    clkoutf_n => clkoutf_n,
    dps_done => dps_done,
    apb_ready => apb_ready,
    apb_rdata => apb_rdata,
    clkin => clkin,
    pll_pd => pll_pd,
    clkoutpd => clkoutpd,
    clk4phasepd => clk4phasepd,
    ssc_en_n => ssc_en_n,
    fbdiv => fbdiv,
    frac => frac,
    clkin1_p => clkin1_p,
    clkin1_n => clkin1_n,
    clkin2 => clkin2,
    clkin2_p => clkin2_p,
    clkin2_n => clkin2_n,
    clkfb => clkfb,
    clkin_sel => clkin_sel,
    pll_pwd => pll_pwd,
    rst => rst,
    clkout0_syn => clkout0_syn,
    clkout1_syn => clkout1_syn,
    clkout2_syn => clkout2_syn,
    clkout3_syn => clkout3_syn,
    clkout4_syn => clkout4_syn,
    clkout5_syn => clkout5_syn,
    clkout6_syn => clkout6_syn,
    clkoutphy_syn => clkoutphy_syn,
    clkoutf_syn => clkoutf_syn,
    clkout0_bufce => clkout0_bufce,
    clkout1_bufce => clkout1_bufce,
    clkout2_bufce => clkout2_bufce,
    clkout3_bufce => clkout3_bufce,
    clkout4_bufce => clkout4_bufce,
    clkout5_bufce => clkout5_bufce,
    clkout6_bufce => clkout6_bufce,
    clkoutf_bufce => clkoutf_bufce,
    dps_clk => dps_clk,
    dps_en => dps_en,
    dps_dir => dps_dir,
    apb_clk => apb_clk,
    apb_rst_n => apb_rst_n,
    apb_addr => apb_addr,
    apb_sel => apb_sel,
    apb_en => apb_en,
    apb_write => apb_write,
    apb_wdata => apb_wdata
  );
