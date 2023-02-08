within MetroscopeModelingLibrary.FlueGases.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.FlueGasesOutletIcon;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidOutlet(Xi_outflow(start = {0.7481,0.1392,0.0525,0.0601,0.0}),redeclare package Medium =
        FlueGasesMedium)                                                             annotation(IconMap(primitivesVisible=false));
end Outlet;
