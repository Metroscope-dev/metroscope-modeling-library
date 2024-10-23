within MetroscopeModelingLibrary.Sensors.RefMoistAir;
model DeltaPressureSensor
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

end DeltaPressureSensor;
