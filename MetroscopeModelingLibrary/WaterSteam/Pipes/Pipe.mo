within MetroscopeModelingLibrary.WaterSteam.Pipes;
model Pipe
  extends MetroscopeModelingLibrary.Icons.Pipes.WaterPipeIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
end Pipe;
