within MetroscopeModelingLibrary.Icons;
package Machines

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-101,-100},{99,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
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
