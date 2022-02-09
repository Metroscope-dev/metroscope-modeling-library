within MetroscopeModelingLibrary.Common.Sensors_NoEq;
model LevelSensor
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-26},{0,-70}},
                                     color={0,0,0}),
        Polygon(points={{-20,-60},{0,-98},{22,-60},{0,-70},{-20,-60}},
            lineColor={0,0,0}),
        Text(
          extent={{-66,66},{64,60}},
          lineColor={28,108,200},
          textString="%name"),
        Text(
          extent={{-8,26},{8,10}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="L"),
        Polygon(points={{-20,54},{-40,14},{-20,-26},{20,-26},{40,14},{20,54},{0,
              54},{-20,54}}, lineColor={0,0,0})}),
                                  Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end LevelSensor;
