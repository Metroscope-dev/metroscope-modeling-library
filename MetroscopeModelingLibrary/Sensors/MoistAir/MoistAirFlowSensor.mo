within MetroscopeModelingLibrary.Sensors.MoistAir;
model MoistAirFlowSensor
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.MoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlowIcon;

  extends Partial.Sensors.FlowSensor(h_0=1e3, P_0=0.9e5,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
    redeclare package Medium = MoistAirMedium, medium_name="MoistAir") annotation(IconMap(primitivesVisible=false));
end MoistAirFlowSensor;
