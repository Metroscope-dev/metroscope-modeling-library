within MetroscopeModelingLibrary.Common.PressureLosses;
model ControlValve "Control valve"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;
public
  MetroscopeModelingLibrary.Common.Units.Cv Cvmax(start=8005.42)
    "Maximum CV (active if mode_caract=0)";
  MetroscopeModelingLibrary.Common.Units.Cv Cv(start=100) "Cv";
  MetroscopeModelingLibrary.Common.Connectors.RealOutput Opening annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,118}),                              iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,182})));
equation
  /* Pressure loss */
  deltaP*Cv*abs(Cv) = -1.733e12*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q, eps)/rhom^2;
  /* Cv as a function of the valve position */
  Cv = Opening*Cvmax;
  Q_in*h_in + Q_out*h_out = 0;
  Q_in*Xi_in =- Q_out*Xi_out;
  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{40,102},{-40,102},{-40,118},{-38,136},{-32,146},{-20,156},{0,
              162},{20,156},{32,146},{38,134},{40,116},{40,102}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{40,102},{-40,102},{0,2}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-100,-38},{0,2},{-100,42},{-100,-40},{-100,-38}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,2},{100,42},{100,-40},{0,2},{0,2}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid)}),
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-40},{100,180}},
        grid={2,2}), graphics={
        Polygon(
          points={{-100,-100},{0,-60},{-100,-20},{-100,-102},{-100,-100}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-60},{100,-20},{100,-102},{0,-60},{0,-60}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,-60},{40,40},{-40,40},{0,-60}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{40,40},{-40,40},{-40,56},{-38,74},{-32,84},{-20,94},{0,100},
              {20,94},{32,84},{38,72},{40,54},{40,40}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid)}),
    Window(
      x=0.07,
      y=0.13,
      width=0.8,
      height=0.77),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end ControlValve;
