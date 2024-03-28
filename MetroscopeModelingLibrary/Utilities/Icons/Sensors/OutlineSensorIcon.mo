within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record OutlineSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5), Text(
          extent={{-100,160},{100,120}},
          textColor={0,0,0},
          textString="%name")}));
end OutlineSensorIcon;
