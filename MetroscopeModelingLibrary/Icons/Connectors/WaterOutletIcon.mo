within MetroscopeModelingLibrary.Icons.Connectors;
partial record WaterOutletIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={Polygon(
          points={{-100,100},{100,0},{-100,-100},{-60,0},{-100,100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end WaterOutletIcon;
