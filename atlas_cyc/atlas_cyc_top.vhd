library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.demistify_config_pkg.all;

-- -----------------------------------------------------------------------

entity atlas_top is
	port (
		CLK12M : in std_logic;
		CLK_X  : in std_logic;
		KEY0   : in std_logic;
		LED    : out std_logic_vector(7 downto 0);
		-- SDRAM
		DRAM_CLK   : out std_logic;
		DRAM_CKE   : out std_logic := '0';
		DRAM_ADDR  : out std_logic_vector(12 downto 0) := (others => '0');
		DRAM_BA    : out std_logic_vector(1 downto 0);
		DRAM_DQ    : inout std_logic_vector(15 downto 0) := (others => 'Z');
		DRAM_LDQM  : out std_logic;
		DRAM_UDQM  : out std_logic;
		DRAM_CS_N  : out std_logic := '1';
		DRAM_WE_N  : out std_logic := '1';
		DRAM_CAS_N : out std_logic := '1';
		DRAM_RAS_N : out std_logic := '1';
		-- HDMI TDMS [or VGA if ATLAS_CYC_VGA = 1]
		TMDS : out std_logic_vector(7 downto 0) := (others => '0');
		-- AUDIO
		SIGMA_R : out std_logic;
		SIGMA_L : out std_logic;
	
		-- PS2
		PS2_KEYBOARD_CLK : inout std_logic;
		PS2_KEYBOARD_DAT : inout std_logic;
		PS2_MOUSE_CLK    : inout std_logic;
		PS2_MOUSE_DAT    : inout std_logic;
		-- UART
		UART_RXD : in std_logic :='1';
		UART_TXD : out std_logic:='1';
		--
		UART_NULL_RXD: in std_logic;
		UART_NULL_TXD: out std_logic;
		-- JOYSTICK 
		JOY1_B2_P9 : in std_logic;
		JOY1_B1_P6 : in std_logic;
		JOY1_UP    : in std_logic;
		JOY1_DOWN  : in std_logic;
		JOY1_LEFT  : in std_logic;
		JOY1_RIGHT : in std_logic;
		-- SHARED PIN_P11: JOY SELECT Output / EAR Input
		JOYX_SEL_EAR : inout std_logic := '0';
		-- SD Card
		SD_CS_N_O : out std_logic := '1';
		SD_SCLK_O : out std_logic := '0';
		SD_MOSI_O : out std_logic := '0';
		SD_MISO_I : in std_logic
	);
end entity;

architecture RTL of atlas_top is

	-- System clocks
	signal locked  : std_logic;
	signal reset_n : std_logic;

	-- SPI signals
	signal sd_clk  : std_logic;
	signal sd_cs   : std_logic;
	signal sd_mosi : std_logic;
	signal sd_miso : std_logic;

	-- internal SPI signals
	signal spi_toguest   : std_logic;
	signal spi_fromguest : std_logic;
	signal spi_ss2       : std_logic;
	signal spi_ss3       : std_logic;
	signal spi_ss4       : std_logic;
	signal conf_data0    : std_logic;
	signal spi_clk_int   : std_logic;

	-- PS/2 Keyboard socket - used for second mouse
	signal ps2_keyboard_clk_in  : std_logic;
	signal ps2_keyboard_dat_in  : std_logic;
	signal ps2_keyboard_clk_out : std_logic;
	signal ps2_keyboard_dat_out : std_logic;

	-- PS/2 Mouse
	signal ps2_mouse_clk_in  : std_logic;
	signal ps2_mouse_dat_in  : std_logic;
	signal ps2_mouse_clk_out : std_logic;
	signal ps2_mouse_dat_out : std_logic;

	signal intercept : std_logic;

	-- Video
	signal vga_red   : std_logic_vector(7 downto 0);
	signal vga_green : std_logic_vector(7 downto 0);
	signal vga_blue  : std_logic_vector(7 downto 0);
	signal vga_hsync : std_logic;
	signal vga_vsync : std_logic;

	-- RS232 serial
	signal rs232_rxd : std_logic;
	signal rs232_txd : std_logic;

	-- IO
	signal joya : std_logic_vector(7 downto 0);
	signal joyb : std_logic_vector(7 downto 0);
	signal joyc : std_logic_vector(7 downto 0);
	signal joyd : std_logic_vector(7 downto 0);




	-- DAC AUDIO     
	signal dac_l : signed(15 downto 0);
	signal dac_r : signed(15 downto 0);


	-- HDMI TDMS signas
	signal clock_vga_s    : std_logic;
	signal clock_dvi_s    : std_logic;
	signal sound_hdmi_l_s : std_logic_vector(15 downto 0);
	signal sound_hdmi_r_s : std_logic_vector(15 downto 0);
	signal tdms_r_s       : std_logic_vector(9 downto 0);
	signal tdms_g_s       : std_logic_vector(9 downto 0);
	signal tdms_b_s       : std_logic_vector(9 downto 0);
	signal tdms_p_s       : std_logic_vector(3 downto 0);
	signal tdms_n_s       : std_logic_vector(3 downto 0);

	-- VGA signals  [ ATLAS_CYC_VGA = 1]
	signal VGA_HS : std_logic;
	signal VGA_VS : std_logic;
	signal VGA_R  : std_logic_vector(1 downto 0);
	signal VGA_G  : std_logic_vector(1 downto 0);
	signal VGA_B  : std_logic_vector(1 downto 0);

	-- VIDEO signals
	signal vga_clk   : std_logic;
	signal hdmi_clk  : std_logic;
	signal vga_blank : std_logic;
	signal vga_x_r   : std_logic_vector(5 downto 0);
	signal vga_x_g   : std_logic_vector(5 downto 0);
	signal vga_x_b   : std_logic_vector(5 downto 0);
	signal vga_x_hs  : std_logic;
	signal vga_x_vs  : std_logic;


	signal clock_50M : std_logic;

	component pll2 is			-- for hdmi output & 50 MHz clock
	    port (
	--  areset : in std_logic;
	    inclk0 : in std_logic;
	    c0 : out std_logic;
	    locked : out std_logic
	  );
	end component;

	-- SHARE PIN P11 EAR IN / JOY SEL OUT  
	signal EAR        : std_logic;
	signal JOYX_SEL_O : std_logic;

begin


	-- SPI
	SD_CS_N_O <= sd_cs;
	SD_MOSI_O <= sd_mosi;
	sd_miso   <= SD_MISO_I;
	SD_SCLK_O <= sd_clk;

	-- External devices tied to GPIOs
	ps2_mouse_dat_in <= ps2_mouse_dat;
	ps2_mouse_dat    <= '0' when ps2_mouse_dat_out = '0' else 'Z';
	ps2_mouse_clk_in <= ps2_mouse_clk;
	ps2_mouse_clk    <= '0' when ps2_mouse_clk_out = '0' else 'Z';

	ps2_keyboard_dat_in <= ps2_keyboard_dat;
	ps2_keyboard_dat    <= '0' when ps2_keyboard_dat_out = '0' else 'Z';
	ps2_keyboard_clk_in <= ps2_keyboard_clk;
	ps2_keyboard_clk    <= '0' when ps2_keyboard_clk_out = '0' else 'Z';



	JOYX_SEL_O   <= '1';
	JOYX_SEL_EAR <= JOYX_SEL_O;
	EAR          <= '0';

	joya <= "11" & JOY1_B2_P9 & JOY1_B1_P6 & JOY1_RIGHT & JOY1_LEFT & JOY1_DOWN & JOY1_UP;
	joyb <= (others => '1');
	joyc <= (others => '1');
	joyd <= (others => '1');



	-- BEGIN VGA ATLAS -------------------  
	VGA_R  <= vga_red(7 downto 6);
	VGA_G  <= vga_green(7 downto 6);
	VGA_B  <= vga_blue(7 downto 6);
	VGA_HS <= vga_hsync;
	VGA_VS <= vga_vsync;

	LED(7) <= vga_red(7);
	LED(6) <= vga_green(7);
	LED(5) <= vga_blue(7);
	

	TMDS(7) <= VGA_R(1);
	TMDS(6) <= VGA_R(0);
	TMDS(5) <= VGA_G(1);
	TMDS(4) <= VGA_G(0);
	TMDS(3) <= VGA_B(1);
	TMDS(2) <= VGA_B(0);
	TMDS(1) <= VGA_VS;
	TMDS(0) <= VGA_HS;

	-- END VGA ATLAS -------------------  

--
--	-- PLL VIDEO / 50 MHz
	pllvideo : pll2
	port map (
		inclk0		=> CLK12M,				
		c0			=> clock_50M,			-- 50 MHz
		locked		=> locked
	);


	guest : component guest_mist
		port map(
			CLOCK_27 => CLK12M,
--			RESET_N => reset_n,
			LED => LED(0),
			--SDRAM
			SDRAM_DQ   => DRAM_DQ,
			SDRAM_A    => DRAM_ADDR,
			SDRAM_DQML => DRAM_LDQM,
			SDRAM_DQMH => DRAM_UDQM,
			SDRAM_nWE  => DRAM_WE_N,
			SDRAM_nCAS => DRAM_CAS_N,
			SDRAM_nRAS => DRAM_RAS_N,
			SDRAM_nCS  => DRAM_CS_N,
			SDRAM_BA   => DRAM_BA,
			SDRAM_CLK  => DRAM_CLK,
			SDRAM_CKE  => DRAM_CKE,
			--UART
			UART_TX  => open,
			UART_RX  => '1',
--			UART_TX  => open,
--			UART_RX  => EAR,   
			--SPI
--			SPI_SD_DI => sd_miso,
			SPI_DO     => spi_fromguest,
			SPI_DI     => spi_toguest,
			SPI_SCK    => spi_clk_int,
			SPI_SS2    => spi_ss2,
			SPI_SS3    => spi_ss3,
--			SPI_SS4    => spi_ss4,
			CONF_DATA0 => conf_data0,
			--VGA
			VGA_HS    => vga_hsync,
			VGA_VS    => vga_vsync,
			VGA_R     => vga_red(7 downto 2),
			VGA_G     => vga_green(7 downto 2),
			VGA_B     => vga_blue(7 downto 2),

			--AUDIO
			DAC_L   => dac_l,
			DAC_R   => dac_r,
			AUDIO_L => SIGMA_L,
			AUDIO_R => SIGMA_R
		);


		-- Pass internal signals to external SPI interface
		sd_clk <= spi_clk_int;

		controller : entity work.substitute_mcu
			generic map(
				sysclk_frequency => 500,
		--		SPI_FASTBIT=>3,			
		--		SPI_INTERNALBIT=>2,		--needed to avoid hungs on the OSD
				debug     => false,
				jtag_uart => false
			)
			port map(
				clk       => clock_50M,	
				reset_in  => '1'   ,	--reset_in when 0
				reset_out => reset_n,		   --reset_out when 0

				-- SPI signals
				spi_miso      => sd_miso,
				spi_mosi      => sd_mosi,
				spi_clk       => spi_clk_int,
				spi_cs        => sd_cs,
				spi_fromguest => spi_fromguest,
				spi_toguest   => spi_toguest,
				spi_ss2       => spi_ss2,
				spi_ss3       => spi_ss3,
				spi_ss4       => spi_ss4,
				conf_data0    => conf_data0,

				-- PS/2 signals
				ps2k_clk_in  => ps2_keyboard_clk_in,
				ps2k_dat_in  => ps2_keyboard_dat_in,
				ps2k_clk_out => ps2_keyboard_clk_out,
				ps2k_dat_out => ps2_keyboard_dat_out,
				ps2m_clk_in  => ps2_mouse_clk_in,
				ps2m_dat_in  => ps2_mouse_dat_in,
				ps2m_clk_out => ps2_mouse_clk_out,
				ps2m_dat_out => ps2_mouse_dat_out,

				-- Buttons
				buttons => (1 => KEY0, others => '1'),

				-- JOYSTICKS
				joy1 => joya,

				-- UART
				rxd       => UART_RXD,
				txd       => UART_TXD,
				intercept => intercept
			);

	end rtl;
