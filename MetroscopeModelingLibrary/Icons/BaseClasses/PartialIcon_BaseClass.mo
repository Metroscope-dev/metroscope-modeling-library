within MetroscopeModelingLibrary.Icons.BaseClasses;
partial model PartialIcon_BaseClass "should not be extended in model"
  extends MetroscopeModelingLibrary.Icons.Colors;
equation
  assert(is_in_partial or is_in_water_steam or is_in_moist_air, "You must define as 'true' at least one of is_in_partial, is_in_water_steam, is_in_moist_air parameters");
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
