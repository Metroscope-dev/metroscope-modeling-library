within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model FuelSink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FuelSinkIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.Fuel.Connectors.FuelInlet C_in,
                                               redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end FuelSink;
