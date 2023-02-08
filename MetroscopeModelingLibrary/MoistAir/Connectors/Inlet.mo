within MetroscopeModelingLibrary.MoistAir.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        MoistAirMedium)                                                            annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid)}));
end Inlet;
