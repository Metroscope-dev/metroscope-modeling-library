within MetroscopeModelingLibrary.Icons.Sensors;
partial model OtherSensorIcon "should be extended in partial base classes"
  annotation (Icon(coordinateSystem(preserveAspectRatio=true),
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,81},{80,-79}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end OtherSensorIcon;
