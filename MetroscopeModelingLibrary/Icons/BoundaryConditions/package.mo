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
          extent={{-74,60},{46,-60}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
      Line(
        points={{54,0},{84,0}},
        color={0,0,0},
        thickness=1),
        Rectangle(
          extent={{45,11},{67,-11}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end BoundaryConditions;
