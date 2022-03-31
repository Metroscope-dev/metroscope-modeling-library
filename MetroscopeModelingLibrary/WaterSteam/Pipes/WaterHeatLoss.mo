within MetroscopeModelingLibrary.WaterSteam.Pipes;
model WaterHeatLoss
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.HeatLoss(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                      redeclare package Medium = WaterSteamMedium);
end WaterHeatLoss;
