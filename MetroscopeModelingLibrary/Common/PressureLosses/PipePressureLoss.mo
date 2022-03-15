within MetroscopeModelingLibrary.Common.PressureLosses;
model PipePressureLoss "Pipe generic pressure loss"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;
  import MetroscopeModelingLibrary.Common.Functions.homotopy;
  import MetroscopeModelingLibrary.Common.Functions.ThermoSquare;

  connector InputReal = input Real;
  connector InputPosition = input Modelica.Units.SI.Position;

  InputReal Kfr(start=10) "Friction pressure loss coefficient";
  InputPosition z1(start=0) "Inlet altitude";
  InputPosition z2(start=0) "Outlet altitude";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaPf(start=1e5) "Singular pressure loss";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaPg(start=0.001e5) "Singular pressure loss";
protected
  constant Modelica.Units.SI.Acceleration g=Modelica.Constants.g_n
    "Gravity constant";
equation
  /* Pressure loss */
  deltaPf = homotopy(-Kfr*ThermoSquare(Q_in, eps)/rhom,
                     -Kfr*ThermoSquare(Q_in_0, eps)/Medium.rho_0, use_homotopy);
  deltaPg = homotopy(-rhom*g*(z2 - z1), -Medium.rho_0*g*(z2 - z1), use_homotopy);
  DP = deltaPf + deltaPg;
  //deltaP = deltaPf + deltaPg;
  //Q_in*h_in + Q_out*h_out = 0;
  W = 0;
  //Q_in*Xi_in =- Q_out*Xi_out;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          textString=
               "K")}),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          textString=
               "K")}),
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
end PipePressureLoss;
