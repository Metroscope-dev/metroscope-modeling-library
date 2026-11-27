within MetroscopeModelingLibrary.SteelworksGases.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = SteelworksGasMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={213,170,255},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid)}));
end Inlet;
