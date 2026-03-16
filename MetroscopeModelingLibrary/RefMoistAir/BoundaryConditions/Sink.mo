within MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
                                                                                                                   redeclare package Medium =
        RefMoistAirMedium)                                                                                                                                       annotation (IconMap(primitivesVisible=
          false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
  Real pds;
  parameter Real k_mair = RefMoistAirMedium.k_mair;
equation
  pds = RefMoistAirMedium.Utilities.pds_pT(P_in, T_in);
  Xi_in = {relative_humidity*k_mair/(P_in/pds - relative_humidity)};

  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,127,127},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,127,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={0,127,127},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={0,127,127},
          thickness=1)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
