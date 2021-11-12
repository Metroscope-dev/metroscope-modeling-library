within MetroscopeModelingLibrary.Common.PressureLosses;
model SingularPressureLoss_NoIcon "Singular pressure loss"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;

  connector InputReal = input Real;

  InputReal Kfr(start=1.e3) "Pressure loss coefficient";
equation
  /* Pressure loss */
  deltaP = -Kfr*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q, eps)/rhom;
  Q_in*h_in + Q_out*h_out = 0;
  Q_in*Xi_in =- Q_out*Xi_out;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Window(
      x=0.09,
      y=0.2,
      width=0.66,
      height=0.69),
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
end SingularPressureLoss_NoIcon;
