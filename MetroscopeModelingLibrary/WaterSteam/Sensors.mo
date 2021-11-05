within MetroscopeModelingLibrary.WaterSteam;
package Sensors
  model WaterFlowSensor
    replaceable package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.FlowSensor(redeclare
        package Medium =
          WaterSteamMedium);
  end WaterFlowSensor;

  model WaterTemperatureSensor
    replaceable package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor(redeclare
        package
        Medium =
          WaterSteamMedium);
  end WaterTemperatureSensor;

  model WaterPressureSensor
    replaceable package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
        package Medium =
          WaterSteamMedium);
  end WaterPressureSensor;

  model WaterDeltaPressureSensor
    replaceable package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.DeltaPressureSensor(redeclare
        package Medium =
          WaterSteamMedium);
  end WaterDeltaPressureSensor;
end Sensors;
