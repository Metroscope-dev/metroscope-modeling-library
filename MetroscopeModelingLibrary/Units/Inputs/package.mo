within MetroscopeModelingLibrary.Units;
package Inputs
  import MetroscopeModelingLibrary.Units;

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
          lineColor={0,140,72},
          lineThickness=1),
        Line(
          points={{-24,0},{24,0}},
          color={0,140,72},
          thickness=1),
        Rectangle(
          extent={{24,30},{82,-28}},
          lineColor={0,140,72},
          lineThickness=1,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}));
end Inputs;
