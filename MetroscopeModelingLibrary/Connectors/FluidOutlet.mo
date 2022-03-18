within MetroscopeModelingLibrary.Connectors;
connector FluidOutlet
  extends MetroscopeModelingLibrary.Connectors.FluidPort(Q(max=0, start=-500)); // Q out of component is negative
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          lineThickness=1)}));
end FluidOutlet;
