within MetroscopeModelingLibrary.Sensors.RefMoistAir;
model FlowSensor
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  extends Partial.Sensors.FlowSensor(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

end FlowSensor;
