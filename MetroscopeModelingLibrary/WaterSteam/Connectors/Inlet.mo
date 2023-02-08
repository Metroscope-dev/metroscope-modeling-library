within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.WaterInletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        WaterSteamMedium)                                                            annotation(IconMap(primitivesVisible=false));
end Inlet;
