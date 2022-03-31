within MetroscopeModelingLibrary.Sensors.MoistAir;
model MoistAirPressureSensor
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.MoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(h_0=1e3, P_0=0.9e5,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
    redeclare package Medium = MoistAirMedium)  annotation(IconMap(primitivesVisible=false));
end MoistAirPressureSensor;
