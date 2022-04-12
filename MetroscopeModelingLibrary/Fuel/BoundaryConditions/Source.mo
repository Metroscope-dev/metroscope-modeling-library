within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FuelSourceIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end Source;
