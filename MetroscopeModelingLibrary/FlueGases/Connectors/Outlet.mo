within MetroscopeModelingLibrary.FlueGases.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FlueGasesOutletIcon;

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        FlueGasesMedium)                                                             annotation(IconMap(primitivesVisible=false));
end Outlet;
