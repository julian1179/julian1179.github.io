library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.all;



entity VDMA_Read is
	generic(
			HSize	:	natural	:= 1920; -- Number of pixels per line
			VSize	:	natural	:= 1080; -- Number of lines per frame

			stride	:	natural	:= 1920*3; -- Number of memory addresses per line. 3==RGB

			PPC		:	natural	:= 8; -- Pixels per clock
			numBuffers	:	natural	:= 3; -- Number of frames to store in RAM

			frame0_startAddr	:	unsigned(31 downto 0) := x"80000000"
		);
	port(
			clk		:	in std_logic; -- 300 MHz
			rst		:	in std_logic;

			-- AXI ports
			tuser	:	out std_logic;	-- Indicates SOF
			tvalid	:	out std_logic;	-- Indicates there is valid data on the tdata line
			tlast	:	out std_logic;	-- Indicates EOL
			tdata	:	out std_logic_vector((3*8*PPC)-1 downto 0); -- 3*8*PPC = RGB*Byte*PPC
			tready	:	in std_logic;	-- Indicates the receiver is ready to recv data

			-- VDMA status ports
			rdFrame		:	out std_logic_vector(integer(ceil(log2(real(numBuffers))))-1 downto 0); -- current Read Frame index
			wrFrame		:	in std_logic_vector(integer(ceil(log2(real(numBuffers))))-1 downto 0); -- current Write Frame index

			-- RAM ports
			rdAddr	:	out std_logic_vector(31 downto 0); 			-- Address of the first (LSB) byte.
			rdStart	:	out std_logic;								-- Pulse to tell the RAM to read from rdAddr
			rdData	:	in std_logic_vector((3*8*PPC)-1 downto 0);	-- Data read from RAM. We expect to read 3*PPC addresses (8 bits each)
			RAM_rd_available	:	in std_logic;	-- Flag that indicates RAM is available for reading.
			rdComplete			:	in std_logic;	-- Flag that indicates RAM has finished a RD request.
			rdError				:	in std_logic	-- Flag that indicates RAM was unable to complete a RD request.
		);
end VDMA_Read;


architecture Behavioral of VDMA_Read is
	---------------------------------------------------------
	------------------------ Constants ----------------------
	---------------------------------------------------------

	type address32bit is array (0 to numBuffers-1) of unsigned(31 downto 0);
	constant start_addr	:	address32bit	:= (0 => frame0_startAddr,
												1 => frame0_startAddr + (3*HSize*VSize),
												2 => frame0_startAddr + 2*(3*HSize*VSize) );
	
	---------------------------------------------------------
	------------------------- States ------------------------
	---------------------------------------------------------
	type VDMA_READ_STATE_TYPE is (RST_STATE, LOAD_DATA, WAIT_RDY);
	signal state : VDMA_READ_STATE_TYPE := RST_STATE;

	---------------------------------------------------------
	----------------------- Port Signals --------------------
	---------------------------------------------------------
	signal frame				:	unsigned(wrFrame'Length-1 downto 0) := (others <= '0');  -- Start on frame 0
	signal user, valid, last	:	std_logic	:= '0';
	signal data					:	std_logic_vector(rdData'Length-1 downto 0) := (others <= '1');

	-- RAM signals
	signal dataIn		:	std_logic_vector(tdata'Length-1 downto 0) := (others <= '0');
	signal startRead	:	std_logic := '0';

	---------------------------------------------------------
	---------------------- Local Signals --------------------
	---------------------------------------------------------
	signal xInd				:	unsigned(integer(ceil(log2(real(HSize))))-1 downto 0)	:= (others <= '0'); -- Measured in memory spaces (3 spaces per pixel [RGB])
	signal yInd				:	unsigned(integer(ceil(log2(real(VSize))))-1 downto 0)	:= (others <= '0'); -- Measured in Lines


	begin

	-- Outputs
	rdFrame		<= std_logic_vector(frame);
	tuser		<= user;
	tvalid		<= valid;
	tlast		<= last;
	tdata		<= data;
	rdStart		<= startRead;
	rdAddr		<= std_logic_vector(start_addr(to_integer(frame)) + (stride*yInd) + xInd);		-- Address of the first (LSB) byte.



	vdmaProcess	:	process(clk, rst, state, tready rdData, RAM_rd_available, rdComplete, rdError)
	begin
		if rst = '1' then
			state 	<= RST_STATE;
			frame	<= (others <= '0');
			user	<= '0';
			valid	<= '0';
			last	<= '0';
			data	<= (others <= '1');
			startRead<= '0';
			xInd 	<= (others <= '0');
			yInd 	<= (others <= '0');

		elsif rising_edge(clk) then
			case state is
				when RST_STATE =>
					-- System just started or received 'rst' signal.
					frame	<= (others <= '0');
					user	<= '0';
					valid	<= '0';
					last	<= '0';
					data	<= (others <= '1');
					xInd 	<= (others <= '0');
					yInd 	<= (others <= '0');

					if (RAM_rd_available = '1') then
						-- When RAM is ready, we will perform the first read of the frame.
						startRead 	<= '1';
						state 		<= LOAD_DATA;
					else
						startRead<= '0';
					end	if;



				when LOAD_DATA =>
					-- We asked RAM to read data. Wait for the data to be ready

					-- Update the EOL signal.
					if (to_integer(xInd)+3*PPC = 3*HSize) then
						last <= '1';
					else
						last <= '0';
					end if;

					-- Update the SOF signal.
					if ((to_integer(yInd) = 0) and (to_integer(xInd) = 0)) then
						user <= '1';
					else
						user <= '0';
					end if;

					if (rdComplete = '1') then
						-- When the data is ready, load it into the Stream
						data	<= rdData;
						valid 	<= '1';

						-- We must immediately prepare the next RAM query to not waste clock cycles.
						if (to_integer(xInd)+3*PPC = 3*HSize) then
							-- We reached the end of the line. Next xInd=0.
							xInd 	<= (others <= '0');

							if (to_integer(yInd) = VSize-1) then
								-- We reached the end of the frame. Next yInd=0.
								yInd <= (others <= '0');

								-- Increase the frame buffer index without intefering with VDMA_WRITE module
								if (to_integer(frame) = numBuffers-1) then
									-- Reached the end of the frame buffer. Next frame=0.
									if (to_integer(unsigned(wrFrame)) = 0) then
										-- The VDMA_WRITE module is currently
										-- using the desired frame. Skip it.
										frame <= (0 <= '1',others <= '0');
									else
										frame <= (others <= '0');
									end if;
								else
									if (to_integer(unsigned(wrFrame)) = to_integer(frame)+1) then
										-- The VDMA_WRITE module is currently
										-- using the desired frame. Skip it.
										if (to_integer(frame) = numBuffers-2) then
											frame <= (others <= '0');
										else
											frame <= frame + 2;
										end if;
									else
										frame <= frame + 1;
									end if;
								end if;

							else
								yInd <= yInd + 1;
							end if;
						else
							xInd <= xInd + 1;
						end if;

						rdStart <= '1'; -- Tell RAM to start reading the next set of data.


						if (tready = '1') then
							-- The receiver was ready before we loaded the data, so it will
							-- be received on this same clock cycle. In this case, we can
							-- just return to this same state and wait for RAM to load the
							-- next packet of data.
							state <= LOAD_DATA;

						else
							-- The receiver was not ready when we updated the data stream
							-- so we must now wait for it to be ready.
							state <= WAIT_RDY;
						end if;
								
					else
						startRead<= '0';
						valid <= '0';
					end if;



				when WAIT_RDY =>
					-- tdata is currently valid, but we need to wait for the receiver to confirm
					-- that they have indeed received it ('tready' = '1')
					startRead<= '0';
					if (tready = '1') then
						-- The receiver has received the data and we can proceed to load the next
						-- packet.
						state <= LOAD_DATA;
					end if;



				when others =>
					-- Possible error state. We should never reach here.
					state 	<= RST_STATE;
					frame	<= (others <= '0');
					ready	<= '0';
					sync_err<= '1'; -- set to 1 because we should never ever get here.
					xInd 	<= (others <= '0');
					yInd 	<= (others <= '0');
					dataIn	<= (others <= '0');
					saveData<= '0';

			end case;
		end if;
	end process;



end Behavioral;
