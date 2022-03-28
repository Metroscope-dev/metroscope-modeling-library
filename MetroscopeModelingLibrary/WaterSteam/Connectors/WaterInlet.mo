within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterInlet
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = WaterSteamMedium);
end WaterInlet;
