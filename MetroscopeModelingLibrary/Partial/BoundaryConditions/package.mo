within MetroscopeModelingLibrary.Partial;
package BoundaryConditions

annotation (Icon(graphics={
      Rectangle(
        lineColor={215,215,215},
        fillColor={255,255,255},
        fillPattern=FillPattern.HorizontalCylinder,
        extent={{-100,-100},{100,100}},
        radius=25),
      Rectangle(
        lineColor={215,215,215},
        extent={{-100,-100},{100,100}},
        radius=25),
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(points={{40,0},{90,0},{72,10}}),
        Ellipse(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
        lineThickness=1),
        Line(points={{90,0},{72,-10}}),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0)}));
end BoundaryConditions;
