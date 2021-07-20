within MetroscopeModelingLibrary.Common.Sensors_NoEq;
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
