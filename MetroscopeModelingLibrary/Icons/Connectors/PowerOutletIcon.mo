within MetroscopeModelingLibrary.Icons.Connectors;
partial record PowerOutletIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={244,125,35},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PowerOutletIcon;
