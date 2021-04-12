within MetroscopeModelingLibrary.Common.Sensors.BaseSensors;
model BaseDifferenceSensor
  "Partial component to model a sensor that measures a potential difference"
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium "Medium in the sensor";
  Connectors.FluidOutlet        C_out(redeclare package Medium =
     Medium)
    annotation (Placement(transformation(extent={{-10,-70},{10,-50}}),
        iconTransformation(extent={{-10,-70},{10,-50}})));
  Connectors.FluidInlet        C_in(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,50},{10,70}}),
        iconTransformation(extent={{-10,50},{10,70}})));
equation
 C_in.Q = 0;
 C_out.Q = 0;
 C_in.H = 0;
 C_out.H = 0;
 C_in.Qi = zeros(Medium.nXi);
 C_out.Qi = zeros(Medium.nXi);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},
            {60,60}}), graphics={
        Line(
          points={{-38,-32},{8,-78},{34,-42}},
          color={0,0,0},
          pattern=LinePattern.None),
        Ellipse(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,0}),
        Line(
          points={{-38,-32},{8,-78},{34,-42}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,50},{0,40}}, color={0,0,0}),
        Line(points={{0,-50},{0,-40}}, color={0,0,0}),
        Line(
          points={{-46,20},{-46,-60},{-56,-40}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-46,60},{-46,-60},{-36,-40}},
          color={0,0,0},
          thickness=0.5)}),     Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-60,-60},{60,60}}), graphics={
        Line(
          points={{-38,-32},{8,-78},{34,-42}},
          color={0,0,0},
          pattern=LinePattern.None),
        Ellipse(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,0}),
        Line(
          points={{-38,-32},{8,-78},{34,-42}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,50},{0,40}}, color={0,0,0}),
        Line(points={{0,-50},{0,-40}}, color={0,0,0}),
        Line(
          points={{-50,-20},{-50,60},{-60,40}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-50,-60},{-50,60},{-40,40}},
          color={0,0,0},
          thickness=0.5)}),
    Documentation(info="<html>
<p><b>V1</b> Creation of the component (23/05/2019)</p>
<p>This component can be used to create sensors that measure the difference between two values in a circuit.</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}), Diagram(coordinateSystem(
          preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}));
end BaseDifferenceSensor;
