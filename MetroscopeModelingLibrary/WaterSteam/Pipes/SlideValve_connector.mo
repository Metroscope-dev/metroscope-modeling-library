within MetroscopeModelingLibrary.WaterSteam.Pipes;
model SlideValve_connector
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Pipes.SlideValve(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium)
                                                annotation(IconMap(primitivesVisible=false));

  parameter Real Cv_constant = 200;

  Utilities.Interfaces.GenericReal Cv_signal(start=Cv_constant) annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,162})));
equation
Cv_signal = Cv;
  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,
              162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{-100,-40},{0,2},{-100,42},{-100,-40},{-100,-40}},
          lineColor={0,0,255},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,255},
          fillColor={28,108,200},
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

end SlideValve_connector;
