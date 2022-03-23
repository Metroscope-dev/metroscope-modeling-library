within MetroscopeModelingLibrary.Connectors;
partial connector FluidOutlet
  extends MetroscopeModelingLibrary.Connectors.FluidPort(Q(max=0, start=-500)); // Q out of component is negative
  annotation (Icon(coordinateSystem(extent={{80,-100},{100,-80}}),
                   graphics={
        Rectangle(
          extent={{80,-80},{100,-100}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(extent={{80,-100},{100,-80}})));
end FluidOutlet;
