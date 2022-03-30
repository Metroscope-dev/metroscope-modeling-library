within MetroscopeModelingLibrary.WaterSteam.Pipes;
model WaterControlValve
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.ControlValve(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                          redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                          redeclare package Medium = WaterSteamMedium);
end WaterControlValve;
