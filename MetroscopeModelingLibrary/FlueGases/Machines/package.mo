within MetroscopeModelingLibrary.FlueGases;
package Machines
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
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Line(points={{-32,0},{30,0}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,20}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,-20}},
        color={0,0,0},
        thickness=1),
        Rectangle(
          extent={{-76,13},{-50,-13}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{50,12},{74,-12}},
          lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Machines;
