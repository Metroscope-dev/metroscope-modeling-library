within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model FuelSource
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FuelSourceIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelOutlet C_out,
                                                 redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end FuelSource;
