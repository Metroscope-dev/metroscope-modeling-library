within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.MoistAirSourceIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out, redeclare
      package                                                                                                                            Medium =
        MoistAirMedium)                                                                                                                                           annotation (IconMap(primitivesVisible=false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);
end Source;
