within MetroscopeModelingLibrary.MoistAir.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Icons.Connectors.MoistAirInletIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        MoistAirMedium)                                                            annotation(IconMap(primitivesVisible=false));
end Inlet;
