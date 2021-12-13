within MetroscopeModelingLibrary.Common.Connectors;
connector FluidOutlet
  extends MetroscopeModelingLibrary.Common.Connectors.FluidPort;
  annotation (defaultComponentName="C_out",
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}),                                       graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={63,81,181},
          fillPattern=FillPattern.Solid,
          lineColor={63,81,181}),    Rectangle(
          extent={{-80,80},{80,-80}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),                           Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
                                                    graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={63,81,181},
          fillPattern=FillPattern.Solid,
          lineColor={63,81,181}), Rectangle(
          extent={{-80,80},{80,-80}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-150,150},{150,190}},
          lineColor={0,0,0},
          textString="%name",
          fontSize=20)}));
end FluidOutlet;
