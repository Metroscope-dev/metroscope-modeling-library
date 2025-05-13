within MetroscopeModelingLibrary.Sensors_Control.Fuel;
model TemperatureSensor
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  import MetroscopeModelingLibrary.Utilities.Types;

  extends Partial.Sensors_Control.TemperatureSensor(
    medium=Types.Medium.Fuel, line=Types.Line.Main, pressure_level=Types.PressureLevel.IP, plant=Types.Plant.CCGT,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

end TemperatureSensor;
