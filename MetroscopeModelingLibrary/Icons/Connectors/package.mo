within MetroscopeModelingLibrary.Icons;
partial package Connectors
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Line(
          points={{-26,0},{22,0}},
          color={102,102,102},
          thickness=1),
        Rectangle(
          extent={{-76,26},{-26,-24}},
          lineColor={102,102,102},
          lineThickness=1,
          fillColor={102,102,102},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{22,30},{80,-28}},
          lineColor={102,102,102},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0)}));
end Connectors;
