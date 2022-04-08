within MetroscopeModelingLibrary.FlueGases;
package Connectors
  
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
        Rectangle(
          extent={{-74,26},{-24,-24}},
          lineColor={175,175,175},
          lineThickness=1,
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(
          points={{-24,0},{24,0}},
          color={175,175,175},
          thickness=1),
        Rectangle(
          extent={{24,30},{82,-28}},
          lineColor={175,175,175},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}));
end Connectors;
