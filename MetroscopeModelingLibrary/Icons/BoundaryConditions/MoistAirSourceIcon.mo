within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record MoistAirSourceIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end MoistAirSourceIcon;
