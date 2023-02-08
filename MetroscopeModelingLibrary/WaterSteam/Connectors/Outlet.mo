within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        WaterSteamMedium)                                                             annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
