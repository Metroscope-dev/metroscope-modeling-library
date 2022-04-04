within MetroscopeModelingLibrary.Power;
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
          extent={{-76,58},{44,-62}},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(points={{44,0},{78,0},{64,10}}),
        Line(points={{64,-10},{78,0}})}));
end BoundaryConditions;
