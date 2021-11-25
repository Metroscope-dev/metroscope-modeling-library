within MetroscopeModelingLibrary.WaterSteam.Sensors;
  model WaterPressureSensor
    replaceable package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
        package Medium =
          WaterSteamMedium);
  end WaterPressureSensor;