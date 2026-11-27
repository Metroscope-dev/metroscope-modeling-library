within MetroscopeModelingLibrary.SteelworksGases.Pipes;
model ControlValve
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.Pipes.ControlValve(
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium,
    Q_0 = 15, rho_0=18.5) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,0},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,0},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,0},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,0},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end ControlValve;
