within MetroscopeModelingLibrary.Utilities.Icons.BaseClasses;
partial record BaseClassIcon "should be extended in partial base classes"
  annotation (Icon(graphics={   Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1), Text(
          extent={{-66,28},{66,-20}},
          textColor={0,0,0},
          textString="%name")}));
end BaseClassIcon;
