within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterInlet
  package WaterSteam = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = WaterSteam);
end WaterInlet;
