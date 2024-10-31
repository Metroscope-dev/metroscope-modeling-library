within MetroscopeModelingLibrary.RefMoistAir.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        RefMoistAirMedium)                                                             annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,127,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
