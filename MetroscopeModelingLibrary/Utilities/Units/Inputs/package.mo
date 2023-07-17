within MetroscopeModelingLibrary.Utilities.Units;
package Inputs
  import MetroscopeModelingLibrary.Utilities.Units;

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
        Line(
          points={{-48,0},{0,0}},
          color={0,140,72},
          thickness=1),
        Rectangle(
          extent={{0,30},{58,-28}},
          lineColor={0,140,72},
          lineThickness=1,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid)}));
end Inputs;
