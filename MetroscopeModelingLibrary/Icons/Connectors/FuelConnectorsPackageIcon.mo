within MetroscopeModelingLibrary.Icons.Connectors;
partial record FuelConnectorsPackageIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  extends MetroscopeModelingLibrary.Icons.PackageIcon;
  annotation (Icon(graphics={
        Rectangle(
          extent={{20,30},{78,-28}},
          lineColor={213,213,0},
          lineThickness=1,
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-28,0},{20,0}},
          color={213,213,0},
          thickness=1),
        Rectangle(
          extent={{-78,26},{-28,-24}},
          lineColor={213,213,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end FuelConnectorsPackageIcon;
