within MetroscopeModelingLibrary.WaterSteam.Sensors;
model WaterDeltaPressureSensor
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.DeltaPressureSensor(redeclare
      package Medium =
        WaterSteamMedium);
end WaterDeltaPressureSensor;
