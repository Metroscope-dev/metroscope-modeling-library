within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  Utilities.Units.NegativePower W_out;
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_out.W;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end Source;
