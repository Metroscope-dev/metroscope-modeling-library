within MetroscopeModelingLibrary.Sensors.WaterSteam;
model DeltaPressureSensor
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));
end DeltaPressureSensor;
