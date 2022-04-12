within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.MoistAirSinkIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in, redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=
          false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_in[1] = MoistAirMedium.massFraction_pTphi(P_in, T_in, relative_humidity);
end Sink;
