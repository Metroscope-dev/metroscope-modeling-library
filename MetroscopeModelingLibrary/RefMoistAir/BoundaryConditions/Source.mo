within MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
                                                                                                                       redeclare package Medium =
        RefMoistAirMedium)                                                                                                                                           annotation (IconMap(primitivesVisible=false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
  Real pds;
  parameter Real k_mair = RefMoistAirMedium.k_mair;

equation
  pds = RefMoistAirMedium.Utilities.pds_pT(P_out, T_out);
  Xi_out = {relative_humidity*k_mair/(P_out/pds - relative_humidity)};

  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end Source;
