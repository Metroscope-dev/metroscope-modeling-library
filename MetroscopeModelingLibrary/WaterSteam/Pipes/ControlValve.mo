within MetroscopeModelingLibrary.WaterSteam.Pipes;
model ControlValve
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.ControlValve(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium);
end ControlValve;
