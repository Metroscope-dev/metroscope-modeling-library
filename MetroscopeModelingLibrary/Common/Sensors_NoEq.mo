within MetroscopeModelingLibrary.Common;
package Sensors_NoEq
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

  model DeltaPSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{-36,0},{-76,0}},
                                       color={0,0,0}),
          Polygon(points={{21,19},{1,-19},{-21,19},{1,9},{21,19}},
              lineColor={0,0,0},
            origin={-85,1},
            rotation=-90),
          Text(
            extent={{-62,72},{68,48}},
            lineColor={28,108,200},
            textString="%name",
            fontSize=6),
          Rectangle(extent={{-36,40},{44,-40}}, lineColor={0,0,0}),
          Line(points={{84,0},{44,0}}, color={0,0,0}),
          Polygon(points={{-21,19},{-1,-19},{21,19},{-1,9},{-21,19}},
              lineColor={0,0,0},
            origin={93,1},
            rotation=90),
          Text(
            extent={{-24,28},{36,-24}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textStyle={TextStyle.Bold},
            textString="P",
            fontSize=6)}),          Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end DeltaPSensor;

  model FlowSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-30},{0,-70}},
                                       color={0,0,0}),
          Polygon(points={{-20,-62},{0,-100},{22,-62},{0,-72},{-20,-62}},
              lineColor={0,0,0}),
          Text(
            extent={{-64,68},{64,62}},
            lineColor={28,108,200},
            textString="%name"),
          Text(
            extent={{-6,20},{6,10}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textStyle={TextStyle.Bold},
            textString="Q"),
          Ellipse(
            extent={{-40,50},{40,-30}},
            lineColor={0,0,0})}),   Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end FlowSensor;
end Sensors_NoEq;
