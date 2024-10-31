within MetroscopeModelingLibrary.RefMoistAir;
package Pipes

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
          radius=25.0),        Rectangle(
          extent={{-48,33},{48,-37}},
          lineColor={0,127,127},
          fillColor={0,160,160},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,11},{-36,-15}},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
        pattern=LinePattern.None,
          lineColor={0,127,127}),
        Rectangle(
          extent={{36,10},{60,-14}},
          lineColor={0,127,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Pipes;
