within MetroscopeModelingLibrary.FlueGases.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FlueGasesInletIcon;

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidInlet(Xi_outflow(start = {0.7481,0.1392,0.0525,0.0601,0.0}),redeclare package Medium =
        FlueGasesMedium)                                                            annotation(IconMap(primitivesVisible=false));
end Inlet;
