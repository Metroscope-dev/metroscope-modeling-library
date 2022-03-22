within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterFluidInlet
  package WaterSteam = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = WaterSteam);
end WaterFluidInlet;
