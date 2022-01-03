within MetroscopeModelingLibrary.Common.Sensors.BaseSensors;
model BaseAbsoluteSensor
  "Partial component to model a sensor that measures a potential variable"
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  Connectors.FluidInlet C_in( redeclare package Medium = Medium, Q(min=0))
    annotation (Placement(transformation(extent={{-10,-50},{10,-30}}),
        iconTransformation(extent={{-10,-50},{10,-30}})));
equation
  C_in.Q = 0;
  C_in.H = 0;
  C_in.Qi = zeros(Medium.nXi);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {40,100}}),                                         graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-40,-40},{40,100}}),
                                      graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}),
    Documentation(info="<html>
<p><b>V1 </b>Creation of the component (23/05/2019)</p>
<p>This component can be used to create sensors that measure the value of a potential in a circuit.</p>
</html>"));
end BaseAbsoluteSensor;
