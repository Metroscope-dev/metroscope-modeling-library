within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Icons.Connectors.WaterInletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
end Inlet;
