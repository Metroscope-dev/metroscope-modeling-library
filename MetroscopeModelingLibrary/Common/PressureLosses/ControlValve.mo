within MetroscopeModelingLibrary.Common.PressureLosses;
model ControlValve "Control valve"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;
  parameter Real CVmax_0 = 8005.4 "Maximum CV (active if mode_caract=0)";

  connector InputCv = input Common.Units.Cv;
  InputCv Cvmax(start=CVmax_0) "Maximum CV (active if mode_caract=0)";
  Common.Units.Cv Cv(start=100) "Cv";
  Modelica.Blocks.Interfaces.RealInput Opening annotation (Placement(
        transformation(extent={{-62,152},{-22,192}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,182})));
equation
  /* Pressure loss */
  //deltaP*Cv*abs(Cv) = -1.733e12*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q, eps)/rhom^2;
  DP*Cv*abs(Cv) = -1.733e12*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q, eps)/rhom^2; // NON LINEAR
  /* Cv as a function of the valve position */
  Cv = homotopy(Opening*Cvmax, Opening*CVmax_0);
  //Q_in*h_in + Q_out*h_out = 0;
  W = 0;
  //Q_in*Xi_in + Q_out*Xi_out = zeros(Medium.nXi); //FlowModel
  //DXi = zeros(Medium.nXi); //FlowModel
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
