within MetroscopeModelingLibrary.Utilities.Icons.BaseClasses;
partial record FuelBaseClassIcon "should be extended in flue gases base classes"
  annotation (Icon(graphics={   Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={213,213,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1), Text(
          extent={{-66,24},{66,-24}},
          textColor={213,213,0},
          textString="%name")}));
end FuelBaseClassIcon;
