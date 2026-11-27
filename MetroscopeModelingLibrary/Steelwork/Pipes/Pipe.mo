within MetroscopeModelingLibrary.Steelwork.Pipes;
model Pipe
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasMedium;
  extends Partial.Pipes.FrictionPipe
                            (
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium) annotation(IconMap(primitivesVisible=false));

  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={213,170,255},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
