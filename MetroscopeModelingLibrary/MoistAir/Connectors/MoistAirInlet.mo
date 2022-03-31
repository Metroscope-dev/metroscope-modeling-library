within MetroscopeModelingLibrary.MoistAir.Connectors;
connector MoistAirInlet
  extends MetroscopeModelingLibrary.Icons.Connectors.MoistAirInletIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MoistAirInlet;
