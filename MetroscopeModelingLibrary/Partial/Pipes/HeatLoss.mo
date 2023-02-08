within MetroscopeModelingLibrary.Partial.Pipes;
partial model HeatLoss
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoPFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,0,255},
          fillColor={85,255,85},
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
