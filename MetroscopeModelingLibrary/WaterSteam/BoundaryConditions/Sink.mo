within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in, redeclare package Medium =
        WaterSteamMedium)                                                                                                                                         annotation (IconMap(primitivesVisible=
         false));
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-16,36},{57,-37}}, color={28,108,200},
          thickness=1),
        Line(points={{-16,-36},{55,35}}, color={28,108,200},
          thickness=1)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
