within MetroscopeModelingLibrary.Utilities.Icons.BaseClasses;
partial record RefMoistAirBaseClassIcon "should be extended in moist air base classes"
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
          lineColor={0,255,128},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1)}));
end RefMoistAirBaseClassIcon;
