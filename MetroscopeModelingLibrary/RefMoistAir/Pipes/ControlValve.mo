within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model ControlValve
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.Pipes.ControlValve(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium, Q_0 = 15, rho_0=1)
                                                annotation(IconMap(primitivesVisible=false));

  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,
              162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,0},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,0},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,0},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,0},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}),
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid)}));
end ControlValve;
