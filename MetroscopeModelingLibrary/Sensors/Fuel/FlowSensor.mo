within MetroscopeModelingLibrary.Sensors.Fuel;
model FlowSensor
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlowIcon;

  extends Partial.Sensors.FlowSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end FlowSensor;
