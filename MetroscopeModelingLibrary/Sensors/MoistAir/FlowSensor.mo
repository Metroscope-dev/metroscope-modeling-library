within MetroscopeModelingLibrary.Sensors.MoistAir;
model FlowSensor
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

  extends Partial.Sensors.FlowSensor(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

end FlowSensor;
