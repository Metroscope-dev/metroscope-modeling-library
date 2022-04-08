within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record FlueGasesSourceIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(points={{40,0},{100,0},{86,10}}),
        Line(points={{86,-10},{100,0}})}));
end FlueGasesSourceIcon;
