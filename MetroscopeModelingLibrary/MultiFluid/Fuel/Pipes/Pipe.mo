within MetroscopeModelingLibrary.MultiFluid.Fuel.Pipes;
model Pipe
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));

  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={213,213,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
