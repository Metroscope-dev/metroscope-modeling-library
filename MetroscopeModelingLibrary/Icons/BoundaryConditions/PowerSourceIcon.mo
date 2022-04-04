within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record PowerSourceIcon
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(points={{40,0},{100,0},{86,10}}),
        Line(points={{86,-10},{100,0}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end PowerSourceIcon;
