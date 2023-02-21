within MetroscopeModelingLibrary.Sensors.WaterSteam;
model FlowSensor
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  extends Partial.Sensors.FlowSensor(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel isoPHFlowModel,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));
end FlowSensor;
