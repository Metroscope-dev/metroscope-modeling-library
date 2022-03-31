within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model MoistAirSource
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSource(P_out_0=0.9e3, h_out_0=1e3, T_out_0=300, Xi_out_0={MoistAirMedium.massFraction_pTphi(P_out_0, T_out_0, relative_humidity_0)},
                                                 redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
                                                 redeclare package Medium = MoistAirMedium);

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);
  annotation (
     Diagram(coordinateSystem(preserveAspectRatio=true)), Icon(graphics={
        Line(points={{62,0},{100,0},{86,10}}),
        Line(points={{86,-10},{100,0}})}));
end MoistAirSource;
