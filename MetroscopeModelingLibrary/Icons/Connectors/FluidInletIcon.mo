within MetroscopeModelingLibrary.Icons.Connectors;
partial record FluidInletIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={95,95,95},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end FluidInletIcon;
