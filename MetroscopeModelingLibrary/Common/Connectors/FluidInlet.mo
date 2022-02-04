within MetroscopeModelingLibrary.Common.Connectors;
connector FluidInlet
  extends MetroscopeModelingLibrary.Common.Connectors.FluidPort(Q(min=0, start=500));
  annotation (defaultComponentName="C_in",
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}),                                       graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={63,81,181},
          fillPattern=FillPattern.Solid,
          lineColor={63,81,181})}),                              Diagram(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
                                                    graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={63,81,181},
          fillPattern=FillPattern.Solid,
          lineColor={63,81,181}),
        Text(
          extent={{-150,150},{150,190}},
          lineColor={0,0,0},
          textString="%name",
          fontSize=20)}));
end FluidInlet;
