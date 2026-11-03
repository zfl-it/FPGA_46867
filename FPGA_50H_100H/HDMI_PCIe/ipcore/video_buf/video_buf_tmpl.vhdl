-- Created by IP Generator (Version 2022.2-SP6.4 build 146967)
-- Instantiation Template
--
-- Insert the following codes into your VHDL file.
--   * Change the_instance_name to your own instance name.
--   * Change the net names in the port map.


COMPONENT video_buf
  PORT (
    wr_clk : IN STD_LOGIC;
    wr_rst : IN STD_LOGIC;
    wr_en : IN STD_LOGIC;
    wr_data : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    wr_full : OUT STD_LOGIC;
    almost_full : OUT STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    rd_rst : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    rd_data : OUT STD_LOGIC_VECTOR(127 DOWNTO 0);
    rd_empty : OUT STD_LOGIC;
    almost_empty : OUT STD_LOGIC;
    clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    wr_byte_en : IN STD_LOGIC;
    wr_water_level : OUT STD_LOGIC_VECTOR(13 DOWNTO 0);
    inject_sbiterr : IN STD_LOGIC;
    inject_dbiterr : IN STD_LOGIC;
    rd_oce : IN STD_LOGIC;
    rd_water_level : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
    data_valid : OUT STD_LOGIC;
    ecc_sbiterr : OUT STD_LOGIC;
    ecc_dbiterr : OUT STD_LOGIC
  );
END COMPONENT;


the_instance_name : video_buf
  PORT MAP (
    wr_clk => wr_clk,
    wr_rst => wr_rst,
    wr_en => wr_en,
    wr_data => wr_data,
    wr_full => wr_full,
    almost_full => almost_full,
    rd_clk => rd_clk,
    rd_rst => rd_rst,
    rd_en => rd_en,
    rd_data => rd_data,
    rd_empty => rd_empty,
    almost_empty => almost_empty,
    clk => clk,
    rst => rst,
    wr_byte_en => wr_byte_en,
    wr_water_level => wr_water_level,
    inject_sbiterr => inject_sbiterr,
    inject_dbiterr => inject_dbiterr,
    rd_oce => rd_oce,
    rd_water_level => rd_water_level,
    data_valid => data_valid,
    ecc_sbiterr => ecc_sbiterr,
    ecc_dbiterr => ecc_dbiterr
  );
