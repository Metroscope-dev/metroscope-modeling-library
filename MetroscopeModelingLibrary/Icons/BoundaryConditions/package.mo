within MetroscopeModelingLibrary.Icons;
package BoundaryConditions

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
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(points={{52,0},{86,0},{72,10}}),
        Line(points={{72,-10},{86,0}}),
        Rectangle(
          extent={{39,11},{61,-11}},
          lineColor={95,95,95},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end BoundaryConditions;
