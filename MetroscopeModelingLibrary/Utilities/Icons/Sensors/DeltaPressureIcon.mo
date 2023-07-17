within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record DeltaPressureIcon
  annotation (Icon(graphics={Text(
          extent={{-100,56},{100,-56}},
          textColor={0,0,0},
          textString="DP"),
        Ellipse(
          extent={{-118,60},{-78,20}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{80,60},{120,20}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={-98,40},
          rotation=90),
        Polygon(
          points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={100,40},
          rotation=180),
        Polygon(
          points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          origin={100,40},
          rotation=90)}));
end DeltaPressureIcon;
