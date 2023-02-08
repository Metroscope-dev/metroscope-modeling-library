within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.BoundaryConditions.FuelSinkIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end Sink;
