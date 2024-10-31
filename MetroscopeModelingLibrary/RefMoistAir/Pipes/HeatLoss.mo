within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model HeatLoss
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.Pipes.HeatLoss(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium)  annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,127,127}),
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
