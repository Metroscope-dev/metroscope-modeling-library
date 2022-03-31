within MetroscopeModelingLibrary.Icons.Connectors;
partial connector PartialIcon_Inlet
  extends MetroscopeModelingLibrary.Icons.Colors;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor=DynamicSelect({28,108,200}, line_color),
          fillColor=DynamicSelect({28,108,200}, inlet_fill_color),
          fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end PartialIcon_Inlet;
