within MetroscopeModelingLibrary.WaterSteam.Sensors;
model WaterPressureSensor
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
      package Medium =
        WaterSteamMedium);
end WaterPressureSensor;
