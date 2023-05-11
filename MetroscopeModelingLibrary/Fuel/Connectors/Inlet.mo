within MetroscopeModelingLibrary.Fuel.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={213,213,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid)}));
end Inlet;
