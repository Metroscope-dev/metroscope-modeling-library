within MetroscopeModelingLibrary.Common;
package Sensors_NoEq
  model TemperatureSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,50},{40,-30}},
            lineColor={0,0,0}),
          Line(points={{0,-30},{0,-70}},
                                       color={0,0,0}),
          Polygon(points={{-20,-60},{0,-98},{22,-60},{0,-70},{-20,-60}},
              lineColor={0,0,0}),
          Text(
            extent={{-66,82},{64,58}},
            lineColor={28,108,200},
            textString="%name"),
          Text(
            extent={{-30,36},{30,-16}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textString="T",
            textStyle={TextStyle.Bold})}),
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
            extent={{-66,82},{64,58}},
            lineColor={28,108,200},
            textString="%name"),
          Rectangle(extent={{-40,50},{40,-30}}, lineColor={0,0,0}),
          Text(
            extent={{-28,38},{32,-14}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textString="P",
            textStyle={TextStyle.Bold})}),
                                    Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end PressureSensor;

  model DeltaPSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(
            points={{-98,-42},{-52,-88},{-26,-52}},
            color={0,0,0},
            pattern=LinePattern.None),
          Line(points={{-36,0},{-76,0}},
                                       color={0,0,0}),
          Line(
            points={{-98,-42},{-52,-88},{-26,-52}},
            color={0,0,0},
            pattern=LinePattern.None),
          Polygon(points={{21,19},{1,-19},{-21,19},{1,9},{21,19}},
              lineColor={0,0,0},
            origin={-85,1},
            rotation=-90),
          Text(
            extent={{-62,72},{68,48}},
            lineColor={28,108,200},
            textString="%name"),
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
            textString="P",
            textStyle={TextStyle.Bold})}),
                                    Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end DeltaPSensor;

  model FlowSensor
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(points={{-20,-30},{-44,-4},{-44,32},{-20,56},{20,56},{44,34},
                {44,-4},{20,-30},{-20,-30}}, lineColor={0,0,0}),
          Line(
            points={{-18,-42},{28,-88},{54,-52}},
            color={0,0,0},
            pattern=LinePattern.None),
          Line(points={{0,-30},{0,-70}},
                                       color={0,0,0}),
          Polygon(points={{-20,-62},{0,-100},{22,-62},{0,-72},{-20,-62}},
              lineColor={0,0,0}),
          Text(
            extent={{-66,82},{64,58}},
            lineColor={28,108,200},
            textString="%name"),
          Text(
            extent={{-30,44},{30,-8}},
            pattern=LinePattern.None,
            fillColor={80,158,47},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0},
            textStyle={TextStyle.Bold},
            textString="Q")}),      Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end FlowSensor;
end Sensors_NoEq;
