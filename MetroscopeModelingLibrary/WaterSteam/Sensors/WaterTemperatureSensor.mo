within MetroscopeModelingLibrary.WaterSteam.Sensors;
model WaterTemperatureSensor
replaceable package WaterSteamMedium =
 MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor(redeclare
      package
         Medium =
   WaterSteamMedium);
end WaterTemperatureSensor;
