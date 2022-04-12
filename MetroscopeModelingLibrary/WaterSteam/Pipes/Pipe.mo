within MetroscopeModelingLibrary.WaterSteam.Pipes;
model Pipe
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium);
end Pipe;
