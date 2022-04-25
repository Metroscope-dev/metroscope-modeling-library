within MetroscopeModelingLibrary.Sensors.Fuel;
model TemperatureSensor
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.TemperatureIcon;

  extends Partial.Sensors.TemperatureSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end TemperatureSensor;
