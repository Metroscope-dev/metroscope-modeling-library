within MetroscopeModelingLibrary.Sensors.Fuel;
model TemperatureSensor
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

  extends Partial.Sensors.TemperatureSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

end TemperatureSensor;
