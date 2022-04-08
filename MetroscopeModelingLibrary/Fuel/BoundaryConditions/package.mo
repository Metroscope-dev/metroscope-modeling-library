within MetroscopeModelingLibrary.Fuel;
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
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
      Line(
        points={{54,-2},{84,-2}},
        color={213,213,0},
        thickness=1),
        Rectangle(
          extent={{42,10},{66,-14}},
          lineColor={213,213,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end BoundaryConditions;
