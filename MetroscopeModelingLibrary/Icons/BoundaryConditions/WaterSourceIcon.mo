within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record WaterSourceIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end WaterSourceIcon;
