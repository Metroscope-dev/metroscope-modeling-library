within MetroscopeModelingLibrary.RefMoistAir.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium =
        RefMoistAirMedium)                                                            annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,255,128},
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid)}));
end Inlet;