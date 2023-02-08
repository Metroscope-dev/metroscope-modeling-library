within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.BoundaryConditions.FlueGasesSourceIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                            annotation (IconMap(
        primitivesVisible=false));
end Source;
