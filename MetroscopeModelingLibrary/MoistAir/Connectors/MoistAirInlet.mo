within MetroscopeModelingLibrary.MoistAir.Connectors;
connector MoistAirInlet
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MoistAirInlet;
