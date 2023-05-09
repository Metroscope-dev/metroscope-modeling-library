within MetroscopeModelingLibrary.Power.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  flow Units.PositivePower W;
  Units.NotUsedVariable dummy(start=0);
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={244,125,35},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid)}));
end Inlet;
