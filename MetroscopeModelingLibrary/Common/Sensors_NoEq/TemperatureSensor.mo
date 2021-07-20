within MetroscopeModelingLibrary.Common.Sensors_NoEq;
model TemperatureSensor
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-30},{0,-70}},
                                     color={0,0,0}),
        Polygon(points={{-20,-60},{0,-98},{22,-60},{0,-70},{-20,-60}},
            lineColor={0,0,0}),
        Text(
          extent={{-66,66},{64,60}},
          lineColor={28,108,200},
          textString="%name"),
        Text(
          extent={{-8,34},{8,18}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="T"),
        Polygon(points={{0,-30},{-46,50},{46,50},{0,-30}}, lineColor={0,0,0})}),
                                  Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end TemperatureSensor;
