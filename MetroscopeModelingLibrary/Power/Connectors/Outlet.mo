within MetroscopeModelingLibrary.Power.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  flow Units.NegativePower W;
  Units.NotUsedVariable dummy(start=0);
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={244,125,35},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Outlet;
