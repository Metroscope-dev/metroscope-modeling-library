within MetroscopeModelingLibrary.Sensors_Control.RefMoistAir;
model PressureSensor
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  extends Partial.Sensors_Control.PressureSensor(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

end PressureSensor;
