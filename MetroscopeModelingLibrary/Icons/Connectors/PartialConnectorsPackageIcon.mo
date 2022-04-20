within MetroscopeModelingLibrary.Icons.Connectors;
partial record PartialConnectorsPackageIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  extends MetroscopeModelingLibrary.Icons.PackageIcon;
  annotation (Icon(graphics={
      Ellipse(
        extent={{-80,78},{80,-82}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,53},{55,-57}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(
          points={{-26,-2},{22,-2}},
          color={0,0,0},
          thickness=1),
        Rectangle(
          extent={{-76,24},{-26,-26}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{22,28},{80,-30}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45,
          origin={0,-2})}));
end PartialConnectorsPackageIcon;
