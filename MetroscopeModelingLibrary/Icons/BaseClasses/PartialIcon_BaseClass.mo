within MetroscopeModelingLibrary.Icons.BaseClasses;
partial model PartialIcon_BaseClass "should not be extended in model"
  extends MetroscopeModelingLibrary.Icons.ModelColors;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false),
              graphics={
                Rectangle(
                  extent={{-100,41},{100,-41}},
                  lineColor=DynamicSelect({28,108,200}, line_color),
                  fillColor=DynamicSelect({255,255,255}, fill_color),
                  fillPattern=FillPattern.Solid,
                  lineThickness=1)}),
                Diagram(coordinateSystem(preserveAspectRatio=false)));
end PartialIcon_BaseClass;
