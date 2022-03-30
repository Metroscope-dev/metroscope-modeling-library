within MetroscopeModelingLibrary.Icons.Connectors;
partial connector PartialIcon_Outlet
  extends MetroscopeModelingLibrary.Icons.Colors;
  annotation (Icon(coordinateSystem(preserveAspectRatio=true),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor=DynamicSelect({28,108,200}, line_color),
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end PartialIcon_Outlet;
