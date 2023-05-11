within MetroscopeModelingLibrary.MoistAir.Pipes;
model HeatLoss
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Pipes.HeatLoss(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium)  annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
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
