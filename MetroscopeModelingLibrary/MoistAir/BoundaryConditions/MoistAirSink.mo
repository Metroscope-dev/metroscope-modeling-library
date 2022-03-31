within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model MoistAirSink
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSink(h_in_0=1e3,
                                               redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
                                               redeclare package Medium = MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={
        Line(points={{-90,0},{-62,0},{-76,10}}),
        Line(points={{-76,-10},{-62,0}})}),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end MoistAirSink;
