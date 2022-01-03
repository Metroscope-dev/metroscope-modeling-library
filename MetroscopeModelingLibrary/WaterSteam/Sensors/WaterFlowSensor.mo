within MetroscopeModelingLibrary.WaterSteam.Sensors;
model WaterFlowSensor
replaceable package WaterSteamMedium =
	MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
extends MetroscopeModelingLibrary.Common.Sensors.FlowSensor(redeclare
	package Medium =
	  WaterSteamMedium);
end WaterFlowSensor;