within MetroscopeModelingLibrary.Icons.Sensors;
partial record AbstractSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Polygon(
          points={{-100,60},{0,-100},{100,60},{-100,60}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{0,-132},{0,-100}}, color={0,0,0}),
        Polygon(
          points={{-10,-128},{0,-148},{10,-128},{0,-132},{-10,-128}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end AbstractSensorIcon;
