within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out, redeclare package Medium =
        MoistAirMedium)                                                                                                                                           annotation (IconMap(primitivesVisible=false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}), Diagram(coordinateSystem(initialScale=0.2)));
end Source;
