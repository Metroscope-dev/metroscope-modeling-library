within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FuelSinkIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end Sink;
