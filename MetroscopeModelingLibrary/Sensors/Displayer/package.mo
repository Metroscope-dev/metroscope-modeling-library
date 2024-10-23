within MetroscopeModelingLibrary.Sensors;
package Displayer

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Line(
          points={{0,72},{0,-80}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{60,-20},{0,-82},{-62,-20}},
          color={238,46,47},
          thickness=1)}));
end Displayer;
