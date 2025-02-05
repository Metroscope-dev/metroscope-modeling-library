within MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={213,213,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
