within MetroscopeModelingLibrary.Icons;
partial package BaseClasses
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
                Rectangle(
                  extent={{-48,27},{48,-27}},
                  lineColor=DynamicSelect({28,108,200}, line_color),
                  fillColor=DynamicSelect({255,255,255}, fill_color),
                  fillPattern=FillPattern.Solid,
                  lineThickness=1)}));
end BaseClasses;
