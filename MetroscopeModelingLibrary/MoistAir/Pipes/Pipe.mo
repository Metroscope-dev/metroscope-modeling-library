within MetroscopeModelingLibrary.MoistAir.Pipes;
model Pipe
  extends MetroscopeModelingLibrary.Utilities.Icons.Pipes.MoistAirPipeIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
