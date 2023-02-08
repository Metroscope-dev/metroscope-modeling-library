within MetroscopeModelingLibrary.MoistAir.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.MoistAirInletIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        MoistAirMedium)                                                            annotation(IconMap(primitivesVisible=false));
end Inlet;
