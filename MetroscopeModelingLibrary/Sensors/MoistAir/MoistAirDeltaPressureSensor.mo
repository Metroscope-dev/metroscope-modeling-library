within MetroscopeModelingLibrary.Sensors.MoistAir;
model MoistAirDeltaPressureSensor
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.OtherSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));
end MoistAirDeltaPressureSensor;
