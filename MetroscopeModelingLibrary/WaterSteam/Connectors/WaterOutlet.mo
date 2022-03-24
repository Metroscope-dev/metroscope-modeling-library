within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterOutlet
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = WaterSteamMedium);
end WaterOutlet;
