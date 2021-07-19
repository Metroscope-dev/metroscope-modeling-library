within MetroscopeModelingLibrary.Common.Sensors_NoEq;
  model PressureSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-30},{0,-70}},
                                       color={0,0,0}),
          Polygon(points={{-20,-60},{0,-98},{22,-60},{0,-70},{-20,-60}},
              lineColor={0,0,0}),
          Text(
            extent={{-64,64},{64,58}},
            lineColor={28,108,200},
            textString="%name"),
          Rectangle(extent={{-40,50},{40,-30}}, lineColor={0,0,0}),
          Text(
            extent={{-6,18},{6,4}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textStyle={TextStyle.Bold},
            textString="P")}),      Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end PressureSensor;