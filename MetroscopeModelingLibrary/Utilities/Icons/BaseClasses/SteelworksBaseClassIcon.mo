within MetroscopeModelingLibrary.Utilities.Icons.BaseClasses;
partial record SteelworksBaseClassIcon
  "should be extended in flue gases base classes"
  annotation (Icon(graphics={   Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={213,170,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1), Text(
          extent={{-66,24},{66,-24}},
          textColor={213,170,255},
          textString="%name")}));
end SteelworksBaseClassIcon;
