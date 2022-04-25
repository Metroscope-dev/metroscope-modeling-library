within MetroscopeModelingLibrary.Sensors.Fuel;
model PressureSensor
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end PressureSensor;
