within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterInlet
  extends MetroscopeModelingLibrary.Icons.Connectors.WaterInletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = WaterSteamMedium) annotation(primitivesVisible=false);
end WaterInlet;
