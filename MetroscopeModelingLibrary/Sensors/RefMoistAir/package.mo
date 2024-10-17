within MetroscopeModelingLibrary.Sensors;
package RefMoistAir

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
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}},
          pattern=LinePattern.None)}));
end RefMoistAir;
