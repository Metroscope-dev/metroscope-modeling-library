within MetroscopeModelingLibrary.Partial.Pipes;
partial model HeatLoss
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoPFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-46,37},{46,-37}},
          textColor={0,0,0},
          textString="W"),
        Line(
          points={{-14,88},{-14,88},{-24,78},{-10,68},{-24,54},{-14,46}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Line(
          points={{0,88},{0,88},{-10,78},{4,68},{-10,54},{0,46}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5),
        Line(
          points={{14,88},{14,88},{4,78},{18,68},{4,54},{14,46}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=0.5)}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatLoss;
