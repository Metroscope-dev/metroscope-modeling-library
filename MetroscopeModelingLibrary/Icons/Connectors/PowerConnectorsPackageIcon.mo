within MetroscopeModelingLibrary.Icons.Connectors;
partial package PowerConnectorsPackageIcon
  //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  extends MetroscopeModelingLibrary.Icons.PackageIcon;
  annotation (Icon(graphics={
        Rectangle(
          extent={{20,30},{78,-28}},
          lineColor={244,125,35},
          lineThickness=1,
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-28,0},{20,0}},
          color={244,125,35},
          thickness=1),
        Rectangle(
          extent={{-78,26},{-28,-24}},
          lineColor={244,125,35},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PowerConnectorsPackageIcon;
