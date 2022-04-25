within MetroscopeModelingLibrary.Sensors.WaterSteam;
model PressureSensor
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.WaterSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));
end PressureSensor;
