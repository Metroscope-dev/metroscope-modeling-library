within MetroscopeModelingLibrary.Sensors.Fuel;
model DeltaPressureSensor
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end DeltaPressureSensor;
