within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        WaterSteamMedium)                                                            annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}));
end Inlet;
