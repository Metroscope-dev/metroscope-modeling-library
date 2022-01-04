within MetroscopeModelingLibrary.FlueGases;
package Sensors
  extends Modelica.Icons.SensorsPackage;
  model WaterFlowSensor
    package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.FlowSensor(redeclare
        package
        Medium =
          FlueGasesMedium);
  end WaterFlowSensor;

  model WaterTemperatureSensor
    package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor(redeclare
        package Medium =
          FlueGasesMedium);
  end WaterTemperatureSensor;

  model WaterPressureSensor
    package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
        package Medium =
          FlueGasesMedium);
  end WaterPressureSensor;

  model WaterDeltaPressureSensor
    package WaterSteamMedium =
        MetroscopeModelingLibrary.WaterSteam.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.DeltaPressureSensor(redeclare
        package Medium =
          FlueGasesMedium);
  end WaterDeltaPressureSensor;
end Sensors;
