within MetroscopeModelingLibrary.Partial;
partial package Connectors
  extends Modelica.Icons.Package;

  annotation (Icon(graphics={
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
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}));
end Connectors;
