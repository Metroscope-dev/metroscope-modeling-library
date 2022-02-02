within MetroscopeModelingLibrary.Common.PressureLosses;
model SingularPressureLoss_NoIcon "Singular pressure loss"
  extends MetroscopeModelingLibrary.Common.Partial.IsoHFlowModel;

  connector InputReal = input Real;

  InputReal Kfr(start=1.e3) "Pressure loss coefficient";
equation
  /* Pressure loss */
  DP = -Kfr*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_in, eps)/rhom;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
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
