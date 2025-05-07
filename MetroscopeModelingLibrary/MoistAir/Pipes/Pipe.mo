within MetroscopeModelingLibrary.MoistAir.Pipes;
model Pipe
    extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Pipes.FrictionPipe
                            (
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
