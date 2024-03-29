within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record AbstractSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Polygon(
          points={{-80,60},{0,-100},{80,60},{-80,60}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Line(points={{0,-132},{0,-100}}, color={0,0,0}),
        Polygon(
          points={{-10,-114},{0,-134},{10,-114},{0,-118},{-10,-114}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,160},{100,120}},
          textColor={0,0,0},
          textString="%name")}));
end AbstractSensorIcon;
