within MetroscopeModelingLibrary.Icons.Sensors;
partial record DeltaPressureIcon
  annotation (Icon(graphics={Text(
          extent={{-100,56},{100,-56}},
          textColor={0,0,0},
          textString="DP"),
        Line(
          points={{-100,120},{100,120}},
          color={0,0,0},
          thickness=1),
        Polygon(
          points={{80,106},{100,120},{80,120},{80,106}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{80,134},{100,120},{80,120},{80,134}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end DeltaPressureIcon;
