within MetroscopeModelingLibrary.MoistAir.Pipes;
model HeightVariationPipe
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAir;
  extends Partial.Pipes.HeightVariationPipe(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,28},{100,-32}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeightVariationPipe;
