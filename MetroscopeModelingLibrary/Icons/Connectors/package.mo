within MetroscopeModelingLibrary.Icons;
partial package Connectors
  extends Icons.PackageIcon;

annotation (Icon(graphics={
        Line(
          points={{-30,-2},{18,-2}},
          color={0,0,0},
          thickness=1),
        Rectangle(
          extent={{-80,24},{-30,-26}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{18,28},{76,-30}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end Connectors;
