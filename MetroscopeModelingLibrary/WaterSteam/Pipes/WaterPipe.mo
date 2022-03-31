within MetroscopeModelingLibrary.WaterSteam.Pipes;
model WaterPipe
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.Pipe(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                             redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                             redeclare package Medium = WaterSteamMedium);
end WaterPipe;
