within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FlueGasesSinkIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                        annotation (IconMap(primitivesVisible=
          false));
end Sink;
