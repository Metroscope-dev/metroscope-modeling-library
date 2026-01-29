within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in, redeclare package Medium =
        MoistAirMedium)                                                                                                                                       annotation (IconMap(primitivesVisible=
          false));

  parameter Real relative_humidity_0(min=0, max=1) = 0.1;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
equation
  Xi_in[1] = MoistAirMedium.massFraction_pTphi(P_in, T_in, relative_humidity);
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={85,170,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={85,170,255},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={85,170,255},
          thickness=1)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
