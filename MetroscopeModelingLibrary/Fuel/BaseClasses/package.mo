within MetroscopeModelingLibrary.Fuel;
package BaseClasses

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
        extent={{-46,49},{46,-47}},
        lineColor={213,213,0},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid,
        lineThickness=1),
      Rectangle(
        extent={{26,18},{60,-16}},
        lineColor={213,213,0},
        lineThickness=1,
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{-64,19},{-28,-17}},
        lineColor={213,213,0},
        lineThickness=1,
        fillColor={213,213,0},
        fillPattern=FillPattern.Solid)}));
end BaseClasses;
