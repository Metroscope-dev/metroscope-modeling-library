within MetroscopeModelingLibrary.WaterSteam.Pipes;
model HeatLoss
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.HeatLoss(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium);
end HeatLoss;
