within MetroscopeModelingLibrary.Utilities.Icons;
package BaseClasses
  extends Icons.PackageIcon;

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

  partial record MoistAirBaseClassIcon "should be extended in moist air base classes"
    annotation (Diagram(graphics={Rectangle(
            extent={{-100,40},{100,-40}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1), Text(
            extent={{-66,24},{66,-24}},
            textColor={85,170,255},
            textString="%name")}),
                                Icon(graphics={
                                  Rectangle(
            extent={{-100,40},{100,-40}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1)}));
  end MoistAirBaseClassIcon;

  partial record FlueGasesBaseClassIcon "should be extended in flue gases base classes"
    annotation (Icon(graphics={   Rectangle(
            extent={{-100,40},{100,-40}},
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1), Text(
            extent={{-66,24},{66,-24}},
            textColor={95,95,95},
            textString="%name")}));
  end FlueGasesBaseClassIcon;

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
  annotation (Icon(graphics={
                Rectangle(
                  extent={{-48,27},{48,-27}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineThickness=1)}));
end BaseClasses;
