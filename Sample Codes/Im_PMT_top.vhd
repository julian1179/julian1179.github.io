library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

Library UNISIM;
use UNISIM.vcomponents.all;

entity Im_PMT_top is
	generic(
			xSize	: natural := 61; 
			ySize	: natural := 61;

			bitDepth		: natural := 8
		);
    Port ( CLK1_125_N,CLK1_125_P          : in STD_LOGIC;
           UART_RXD : in STD_LOGIC;
           UART_TXD : out STD_LOGIC);
end Im_PMT_top;

architecture Behavioral of Im_PMT_top is

	
	component TX_control
		generic(
			xSize	: natural :=xSize; 
			ySize	: natural :=ySize;

			bitDepth		: natural := bitDepth
			);
		port(
			clk		:	in std_logic;
			start 	:	in std_logic;
			im 		:	in std_logic_vector(bitDepth-1 downto 0);
			x_ind	:	out std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0);
			y_ind	:	out std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0);
			data_output	:	out std_logic
			);
	end component;

	component RX_control
		generic(
			xSize	: natural :=xSize; 
			ySize	: natural :=ySize;

			bitDepth		: natural := bitDepth
			);
		port(
			clk			:	in std_logic;
			data_input	:	in std_logic;
			x_ind		:	out std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0);
			y_ind		:	out std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0);
			wr_en		:	out std_logic;
			im 			:	out std_logic_vector(bitDepth-1 downto 0);
			finished 	:	out std_logic
			);
	end component;
	
	component pix_RAM
		generic(
				xSize		: natural := xSize;
				ySize		: natural := ySize;
				
				bitDepth	: natural := bitDepth -- greyscale bitdepth
		);
		port(
				clk 		: in std_logic;
				rst			: in std_logic;
				
				rd_addrx	: in std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0); -- Enough bits to read up to xSize
				rd_addry	: in std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0); -- Enough bits to read up to ySize
				
				wr_addrx	: in std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0);
				wr_addry	: in std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0);
				wr_en		: in std_logic;
				
				data_in		: in std_logic_vector(bitDepth-1 downto 0);
				data_out	: out std_logic_vector(bitDepth-1 downto 0)
		);
	end component;
	
	component PMT_61x61
		generic(
				xSize		: natural := xSize;
				ySize		: natural := ySize
		);
		port(
			x_in	: in std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0); -- Enough bits to read up to xSize
			y_in	: in std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0); -- Enough bits to read up to xSize
			x_out	: out std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0); -- Enough bits to read up to xSize
			y_out	: out std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0) -- Enough bits to read up to xSize
		);
	end component;
	
	signal imSig_RX, imSig_TX : std_logic_vector(bitDepth-1 downto 0);

	signal tx_x, rx_x, tx_x_T : std_logic_vector(integer(ceil(log2(real(xSize))))-1 downto 0);
	signal tx_y, rx_y, tx_y_T : std_logic_vector(integer(ceil(log2(real(ySize))))-1 downto 0);
	
	signal data_input, data_output : STD_LOGIC;
	
	signal readyFlag : std_logic;
	signal rx_write : std_logic;
	signal rstOFF : std_logic := '1';
	signal CLK :std_logic;
begin

	UART_TXD <= data_output;
	data_input <= UART_RXD;
	
	inst_Transform: PMT_61x61 port map(
		x_in	=> tx_x,
		y_in	=> tx_y,
		x_out	=> tx_x_T,
		y_out	=> tx_y_T
	);

	inst_pix_RAM: pix_RAM port map(
		clk			=> CLK,
		rst			=> rstOFF,
		rd_addrx	=> tx_x_T,
		rd_addry	=> tx_y_T,
		wr_addrx	=> rx_x,
		wr_addry	=> rx_y,
		wr_en		=> rx_write,
		data_in		=> imSig_RX,
		data_out	=> imSig_TX
	);

	inst_RX_control: RX_control port map(
		clk 		=> CLK,
		data_input 	=> data_input,
		x_ind		=> rx_x,
		y_ind		=> rx_y,
		wr_en		=> rx_write,
		im 			=> imSig_RX,
		finished 	=> readyFlag
	);
	
	inst_TX_control: TX_control port map(
		clk			=> CLK,
		start 		=> readyFlag,
		im 			=> imSig_TX,
		x_ind		=> tx_x,
		y_ind		=> tx_y,
		data_output	=> data_output
	);	

	IBUFDS_inst : IBUFDS
    port map (
        O => CLK, -- 1-bit output: Buffer output
        I => CLK1_125_P, -- 1-bit input: Diff_p buffer input (connect directly to top-level port)
        IB => CLK1_125_N -- 1-bit input: Diff_n buffer input (connect directly to top-level port)
    );
	
end Behavioral;
