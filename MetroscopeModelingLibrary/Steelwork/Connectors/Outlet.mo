within MetroscopeModelingLibrary.Steelwork.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = SteelworksGasMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={213,170,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
