within MetroscopeModelingLibrary.Sensors.WaterSteam;
model PressureSensor
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;
end PressureSensor;
