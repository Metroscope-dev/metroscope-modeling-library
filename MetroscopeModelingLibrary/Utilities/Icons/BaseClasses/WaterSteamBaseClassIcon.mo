within MetroscopeModelingLibrary.Utilities.Icons.BaseClasses;
partial record WaterSteamBaseClassIcon "should be extended in water steam base classes"
  annotation (Icon(graphics={   Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1), Text(
          extent={{-64,28},{68,-20}},
          textColor={28,108,200},
          textString="%name")}));
end WaterSteamBaseClassIcon;
