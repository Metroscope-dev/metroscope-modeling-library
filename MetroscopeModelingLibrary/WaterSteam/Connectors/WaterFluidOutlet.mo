within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterFluidOutlet
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = WaterSteamMedium);
end WaterFluidOutlet;
