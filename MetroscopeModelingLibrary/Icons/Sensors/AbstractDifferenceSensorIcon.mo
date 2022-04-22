within MetroscopeModelingLibrary.Icons.Sensors;
partial record AbstractDifferenceSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Line(points={{0,-16},{0,16}},    color={0,0,0},
          origin={116,0},
          rotation=90),
        Polygon(
          points={{-10,10},{0,-10},{10,10},{0,6},{-10,10}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={138,0},
          rotation=90),
        Polygon(
          points={{0,100},{-100,0},{0,-100},{100,0},{0,100}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{0,-16},{0,16}},    color={0,0,0},
          origin={-116,0},
          rotation=270),
        Polygon(
          points={{-10,10},{0,-10},{10,10},{0,6},{-10,10}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={-138,0},
          rotation=270)}));
end AbstractDifferenceSensorIcon;
