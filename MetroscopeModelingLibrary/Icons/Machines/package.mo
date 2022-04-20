within MetroscopeModelingLibrary.Icons;
package Machines
  extends Icons.PackageIcon;

  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-32,0},{30,0}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,20}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,-20}},
        color={0,0,0},
        thickness=1)}));
end Machines;
