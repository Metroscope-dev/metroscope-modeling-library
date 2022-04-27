within MetroscopeModelingLibrary.Fuel.Pipes;
model Pipe
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={213,213,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
