within MetroscopeModelingLibrary.MultiFluid.Fuel.Pipes;
model HeatLoss
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Pipes.HeatLoss(
    redeclare MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium,
    Q_0 = 500, rho_0=1)
    annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={95,95,95},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-14,50},{-14,50},{-24,40},{-10,30},{-24,16},{-14,8}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Line(
          points={{0,50},{0,50},{-10,40},{4,30},{-10,16},{0,8}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Line(
          points={{14,50},{14,50},{4,40},{18,30},{4,16},{14,8}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5)}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatLoss;
