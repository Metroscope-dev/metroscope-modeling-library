within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model MoistAirSource
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.MoistAirSourceIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
                                                 redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);
  annotation (
     Diagram(coordinateSystem(preserveAspectRatio=true)), Icon(graphics={
        Line(points={{62,0},{100,0},{86,10}}),
        Line(points={{86,-10},{100,0}})}));
end MoistAirSource;
