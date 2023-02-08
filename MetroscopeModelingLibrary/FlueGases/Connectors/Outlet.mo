within MetroscopeModelingLibrary.FlueGases.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidOutlet(Xi_outflow(start = {0.7481,0.1392,0.0525,0.0601,0.0}),redeclare package Medium =
        FlueGasesMedium)                                                             annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
