within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Source_Pert
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                            annotation (IconMap(
        primitivesVisible=false));
  Modelica.Blocks.Interfaces.RealInput Q_fg annotation (Placement(transformation(extent={{-100,-20},{-60,20}}), iconTransformation(extent={{-100,-20},{-60,20}})));
equation
  Q_fg = - Q_out;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end Source_Pert;
