within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                            annotation (IconMap(
        primitivesVisible=false));
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}), Diagram(coordinateSystem(initialScale=0.2)));
end Source;
