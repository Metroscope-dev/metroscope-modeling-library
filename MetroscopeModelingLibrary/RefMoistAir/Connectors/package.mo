within MetroscopeModelingLibrary.RefMoistAir;
package Connectors
  extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;

  annotation (Icon(graphics={
        Rectangle(
          extent={{20,30},{78,-28}},
          lineColor={0,255,128},
          lineThickness=1,
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-28,0},{20,0}},
          color={0,255,128},
          thickness=1),
        Rectangle(
          extent={{-78,26},{-28,-24}},
          lineColor={0,255,128},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Connectors;