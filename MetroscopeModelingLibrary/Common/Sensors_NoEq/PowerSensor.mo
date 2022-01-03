within MetroscopeModelingLibrary.Common.Sensors_NoEq;
model PowerSensor
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,50},{40,-30}},
          lineColor={0,0,0}),
        Line(points={{0,-30},{0,-70}},
                                     color={0,0,0}),
        Polygon(points={{-20,-62},{0,-100},{22,-62},{0,-72},{-20,-62}},
            lineColor={0,0,0}),
        Text(
          extent={{-64,68},{64,62}},
          lineColor={28,108,200},
          textString="%name"),
        Text(
          extent={{-12,18},{14,-6}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="W")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end PowerSensor;
