within MetroscopeModelingLibrary.WaterSteam.Machines;
model WaterPump
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Machines.Pump(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                redeclare package Medium = WaterSteamMedium);
end WaterPump;
