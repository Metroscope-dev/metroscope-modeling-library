within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out, redeclare package Medium =
        WaterSteamMedium)                                                                                                                                             annotation (IconMap(
        primitivesVisible=false));
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}), Diagram(coordinateSystem(initialScale=0.2)));
end Source;
