local COM_Port = 52 -- Change the COM Port number as needed

TXBO = 0 -- tx back of must be multiples of 3; valid values are 0,3,6,9,12,15,18,21

-- Update the MSS and BSS file path in the below variable
local BSS_Path = ([[C:\ti\mmwave_studio_02_00_00_02\rf_eval_firmware\radarss\iwr6xxx_radarss_rprc.bin]])
local MSS_Path = ([[C:\ti\mmwave_studio_02_00_00_02\rf_eval_firmware\masterss\iwr6xxx_masterss_rprc.bin]])



WriteToLog("LUA Script for System Check\n", "blue")
RSTD.Sleep(1000)

if (0 == ar1.SOPControl(2)) then
	WriteToLog("SOP Reset Success\n", "green")
else
	WriteToLog("SOP Reset Failure\n", "red")
end

RSTD.Sleep(1000)

if (0 == ar1.Connect(COM_Port,921600,1000)) then
	WriteToLog("RS232 Connect Success\n", "green")
else
	WriteToLog("RS232 Connect Failure\n", "red")
end

RSTD.Sleep(1000)

if (ar1.DownloadBSSFw(BSS_Path)) then
	WriteToLog("BSS FW Download Success\n", "green")
else
	WriteToLog("BSS FW Download Failure\n", "red")
end

RSTD.Sleep(1000)

if (0 == ar1.DownloadMSSFw(MSS_Path)) then
	WriteToLog("MSS FW Download Success\n", "green")
else
	WriteToLog("MSS FW Download Failure\n", "red")
end
		
RSTD.Sleep(1000)

if (0 == ar1.PowerOn(1, 1000, 0, 0)) then
	WriteToLog("PowerOn Success\n", "green")
else
	WriteToLog("PowerOn Failure\n", "red")
	session:destroy();
end

RSTD.Sleep(1000)

if (0 == ar1.RfEnable()) then
	WriteToLog("RfEnable Success\n", "green")
else
	WriteToLog("RfEnable Failure\n", "red")
end

RSTD.Sleep(1000)


if (0 == ar1.ChanNAdcConfig(1, 1, 1, 1, 1, 1, 1, 2, 2, 0)) then
	WriteToLog("ChanNAdcConfig Success\n", "green")
else
	WriteToLog("ChanNAdcConfig Failure\n", "red")
end

RSTD.Sleep(1000)

ar1.RfLdoBypassConfig(0x0)

if (0 == ar1.LPModConfig(0, 0)) then
	WriteToLog("LowPowerConfig Success\n", "green")
else
	WriteToLog("LowPowerConfig Failure\n", "red")
end

RSTD.Sleep(1000)



if (0 == ar1.RfInit()) then
	WriteToLog("RfInit Success\n", "green")
else
	WriteToLog("RfInit Failure\n", "red")
end

RSTD.Sleep(1000)

if (0 == ar1.DataPathConfig(1, 0, 0)) then
	WriteToLog("DataPathConfig Success\n", "green")
else
	WriteToLog("DataPathConfig Failure\n", "red")
end

RSTD.Sleep(1000)

if (0 == ar1.LvdsClkConfig(1, 0)) then
	WriteToLog("LvdsClkConfig Success\n", "green")
else
	WriteToLog("LvdsClkConfig Failure\n", "red")
end

RSTD.Sleep(1000)

if (0 == ar1.LVDSLaneConfig(0, 1, 1, 0, 0, 1, 0, 0)) then
	WriteToLog("LVDSLaneConfig Success\n", "green")
else
	WriteToLog("LVDSLaneConfig Failure\n", "red")
end
RSTD.Sleep(1000)



if (0 == ar1.ProfileConfig(0, 61.1, 2, 3.4, 18, TXBO, TXBO, TXBO, 0, 0, 0, 16.693, 1, 128, 10000, 0, 131072, 30)) then
	WriteToLog("Profile Config Success\n", "green")
else
	WriteToLog("Profile Config  Failure\n", "red")
end


RSTD.Sleep(1000)
if (0 == ar1.ChirpConfig(0, 0, 0, 0, 0, 0, 0, 1, 1, 1)) then
	WriteToLog("Chirp Config Success\n", "green")
else
	WriteToLog("Chirp Config  Failure\n", "red")
end

RSTD.Sleep(1000)
if (0 == ar1.FrameConfig(0, 0, 0, 128, 16 , 0, 1)) then
	WriteToLog("Frame Config Success\n", "green")
else
	WriteToLog("Frame Config  Failure\n", "red")
end

RSTD.Sleep(1000)



if (0 == ar1.StartFrame()) then
	WriteToLog("Frame Start Success\n", "green")
else
	WriteToLog("Frame Start Failed\n", "red")
end


-- End of Script