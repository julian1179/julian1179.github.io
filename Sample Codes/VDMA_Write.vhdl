library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.all;



entity VDMA_Write is
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
			tuser	:	in std_logic;	-- Indicates SOF
			tvalid	:	in std_logic;	-- Indicates there is valid data on the tdata line
			tlast	:	in std_logic;	-- Indicates EOL
			tdata	:	in std_logic_vector((3*8*PPC)-1 downto 0); -- 3*8*PPC = RGB*Byte*PPC
			tready	:	out std_logic;	-- Indicates we are ready to recv data

			-- VDMA status ports
			rdFrame		:	in std_logic_vector(integer(ceil(log2(real(numBuffers))))-1 downto 0); -- current Read Frame index
			wrFrame		:	out std_logic_vector(integer(ceil(log2(real(numBuffers))))-1 downto 0); -- current Write Frame index
			sync_error	:	out std_logic; -- Flag that goes to '1' if 'tlast' is not sync'd to our x-index counter



			-- RAM ports
			wrAddr	:	out std_logic_vector(31 downto 0); 			-- Address of the first (LSB) byte.
			wrData	:	out std_logic_vector((3*8*PPC)-1 downto 0); -- Data to be written to RAM
			wrStart	:	out std_logic;								-- Pulse to tell the RAM to save wrData.
			RAM_wr_available	:	in std_logic;	-- Flag that indicates RAM is available for writing.
			wrComplete			:	in std_logic;	-- Flag that indicates RAM has finished a WR request.
			wrError				:	in std_logic	-- Flag that indicates RAM was unable to complete a WR request.
				-- NOTE:	 This module sends out all 192 bits that were received. the logic to
				--			convert the 192 bits into 24 bytes and store them in RAM must be
				--			included on the top layer.
				--			 This choice was made because the DDR4 has a 'burst mode' that can
				--			write multiple bytes at once, which is faster than writing one byte
				--			at a time. 'wrAddr' is the address of the first (LSB) byte.
		);
end VDMA_Write;


architecture Behavioral of VDMA_Write is
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
	type VDMA_WRITE_STATE_TYPE is (RST_STATE, INIT_STATE, ERROR_STATE, WAIT_SOF, STDBY_WAIT, RECVd_EOL_DATA, RECVd_SOF_DATA, RECVd_DATA);
	signal state : VDMA_WRITE_STATE_TYPE := RST_STATE;

	---------------------------------------------------------
	----------------------- Port Signals --------------------
	---------------------------------------------------------
	signal	frame			:	unsigned(wrFrame'Length-1 downto 0) := (0 <= '1',others <= '0'); -- Start on frame 1
	signal	ready, sync_err	:	std_logic	:= '0';

	-- RAM signals
	signal dataIn			:	std_logic_vector(tdata'Length-1 downto 0) := (others <= '0');
	signal saveData			:	std_logic := '0';

	---------------------------------------------------------
	---------------------- Local Signals --------------------
	---------------------------------------------------------
	signal xInd				:	unsigned(integer(ceil(log2(real(HSize))))-1 downto 0)	:= (others <= '0'); -- Measured in memory spaces (3 spaces per pixel [RGB])
	signal yInd				:	unsigned(integer(ceil(log2(real(VSize))))-1 downto 0)	:= (others <= '0'); -- Measured in Lines


	begin

	-- Outputs
	wrFrame		<= std_logic_vector(frame);
	tready		<= ready;
	sync_error	<= sync_err;
	wrData		<= dataIn;		-- Data to be written to RAM
	wrStart		<= saveData;	-- Pulse to tell the RAM to save wrData.
	wrAddr		<= std_logic_vector(start_addr(to_integer(frame)) + (stride*yInd) + xInd);		-- Address of the first (LSB) byte.


	vdmaProcess	:	process(clk, rst, state, tuser, tdata, tvalid, tlast, xInd, yInd, frame, RAM_wr_available, wrComplete, wrError)
	begin
		if rst = '1' then
			state 	<= RST_STATE;
			frame	<= (0 <= '1',others <= '0'); -- Start on frame 1
			ready	<= '0';
			sync_err<= '0';
			xInd 	<= (others <= '0');
			yInd 	<= (others <= '0');
			dataIn	<= (others <= '0');
			saveData<= '0';

		elsif rising_edge(clk) then
			case state is
				when RST_STATE =>
					-- System just started or received 'rst' signal.
					state 	<= INIT_STATE;
					frame	<= (0 <= '1',others <= '0'); -- Start on frame 1
					ready	<= '0';
					sync_err<= '0';
					xInd 	<= (others <= '0');
					yInd 	<= (others <= '0');
					dataIn	<= (others <= '0');
					saveData<= '0';

				when ERROR_STATE =>
					-- System out of sync. Reset but leave 'sync_err' = '1'.
					state 	<= INIT_STATE;
					frame	<= (0 <= '1',others <= '0'); -- Start on frame 1
					ready	<= '0';
					sync_err<= '1';
					xInd 	<= (others <= '0');
					yInd 	<= (others <= '0');
					dataIn	<= (others <= '0');
					saveData<= '0';

				when INIT_STATE =>
					-- System has started, and the signals are reset. Wait for RAM to be ready.
					-- When RAM is ready, we will set 'ready' to '1' and go to WAIT_SOF
					if (RAM_wr_available = '1') then
						ready 	<= '1';
						state	<= WAIT_SOF;
					end	if;

				when WAIT_SOF =>
					-- As soon as we receive the SOF, set 'ready' to '0' and enter the standby
					-- state. This means that the source (TPG, XGS, etc) has finished
					-- configuring and it is ready to send the first frame.
					if (tuser = '1') then
						state	<= STDBY_WAIT;
						xInd 	<= (others <= '0');
						yInd 	<= (others <= '0');
						ready	<= '0';
					end if;

				when STDBY_WAIT =>
					-- When we enter this state, 'ready' = '0'.
					-- If the system didn't just reset, we told RAM to save the data a couple
					-- 	of clock cycles ago. At this point we need to wait until RAM has finished
					--	saving the data and there is valid data on the line.

					if (wrComplete = '1') then
						-- RAM finished saving the data.
						ready 	<= '1'; -- tell the source that we are ready to recv the data.

						-- Wait for the 'tvalid' signal to go to '1'.
						if (tvalid = '1') then
							-- At this point, there is new pixel data on the 'tdata' line.
							if (tlast = '1') then
								state <= RECVd_EOL_DATA;
							elsif (tuser = '1') then
								state <= RECVd_SOF_DATA;
							else
								state <= RECVd_DATA;
							end if;
							dataIn 	<= tdata; -- capture the data and place it on the output port.
							saveData<= '1'; -- tell RAM to start saving the data.
						end if;
					end if;

				when  RECVd_EOL_DATA =>
					-- This state can only last 1 clk_cycle. It is used to prepare the x,y
					-- indices, the frame index, and check for EOL synchronization errors.
					ready 	<= '0'; -- stop receiving data while we save the previous batch.
					saveData<= '0';

					-- Calculate the next Indices and Frame
					if (to_integer(xInd)+3*PPC = 3*HSize) then
						-- We reached the end of the line, and tlast (EOL) was active.
						state	<= STDBY_WAIT;
						sync_err<= '0';

						xInd 	<= (others <= '0'); -- Next xInd=0.

						if (to_integer(yInd) = VSize-1) then
							-- We reached the end of the frame. Next yInd=0.
							-- (Expect SOF on next recvd_data)
							yInd <= (others <= '0');

							-- Increase the frame buffer index without intefering with VDMA_READ module
								if (to_integer(frame) = numBuffers-1) then
									-- Reached the end of the frame buffer. Next frame=0.
									if (to_integer(unsigned(rdFrame)) = 0) then
										-- The VDMA_READ module is currently
										-- using the desired frame. Skip it.
										frame <= (0 <= '1',others <= '0');
									else
										frame <= (others <= '0');
									end if;
								else
									if (to_integer(unsigned(rdFrame)) = to_integer(frame)+1) then
										-- The VDMA_READ module is currently
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
						state <= ERROR_STATE;
						sync_err <= '1';
					end if;


				when RECVd_SOF_DATA =>
					-- This state can only last 1 clk_cycle. It is used to prepare the x,y
					-- indices and check for SOF synchronization errors.
					ready 	<= '0'; -- stop receiving data while we save the previous batch.
					saveData<= '0';

					if ((to_integer(yInd) = 0) and (to_integer(xInd) = 0)) then
						-- We recv'd SOF at the correct indices
						state<= STDBY_WAIT;
						xInd <= xInd + 3*PPC; -- 3*PPC = RGB*PPC
					else
						state <= ERROR_STATE;
						sync_err <= '1';
					end if;



				when RECVd_DATA =>
					-- This state can only last 1 clk_cycle. It is used to prepare the x,y indices
					ready 	<= '0'; -- stop receiving data while we save the previous batch.
					saveData<= '0';

					-- Calculate the next Indices
					if (to_integer(xInd)+3*PPC = 3*HSize) then
						-- We reached the end of the line, but tlast (EOL) was not active.
						state <= ERROR_STATE;
						sync_err <= '1';
					elsif ((to_integer(yInd) = 0) and (to_integer(xInd) = 0)) then
						-- We reached the start of the next frame, but tuser (SOF) was not active.
						state <= ERROR_STATE;
						sync_err <= '1';
					else
						state<= STDBY_WAIT;
						xInd <= xInd + 3*PPC; -- 3*PPC = RGB*PPC						
					end if;


				when others =>
					-- Possible error state. We should never reach here.
					state 	<= RST_STATE;
					frame	<= (0 <= '1',others <= '0'); -- Start on frame 1
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
