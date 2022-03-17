within MetroscopeModelingLibrary.Common.PressureLosses;
model SingularPressureAndHeatLoss "Singular pressure loss"
  extends MetroscopeModelingLibrary.Common.Partial.FlowModel;
  import MetroscopeModelingLibrary.Common.Functions.ThermoSquare;

  connector InputReal = input Real;

  InputReal Kfr(start=1.e3) "Friction pressure loss coefficient";
  Modelica.Blocks.Interfaces.RealInput W_in annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-32,60}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,60})));
equation
  /* Pressure loss */
  //DP = homotopy(-Kfr*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_in, eps)/rhom,
  //              -Kfr*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_in_0, eps)/rhom);
  DP = -Kfr*ThermoSquare(Q_in, eps)/rhom;
  W = W_in;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{-60,40},{-40,20},{-20,10},{0,8},{20,10},{40,20},{60,40},{-60,
              40}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid), Polygon(
          points={{-60,-40},{-40,-20},{-20,-12},{0,-10},{20,-12},{40,-20},{60,
              -40},{-60,-40}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid)}),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{-60,40},{-40,20},{-20,10},{0,8},{20,10},{40,20},{60,40},{-60,
              40}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid), Polygon(
          points={{-60,-40},{-40,-20},{-20,-12},{0,-10},{20,-12},{40,-20},{60,
              -40},{-60,-40}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Baligh El Hefni</li>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end SingularPressureAndHeatLoss;
