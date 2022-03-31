within MetroscopeModelingLibrary.Sensors.WaterSteam;
model WaterPressureSensor
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.WaterSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
end WaterPressureSensor;
