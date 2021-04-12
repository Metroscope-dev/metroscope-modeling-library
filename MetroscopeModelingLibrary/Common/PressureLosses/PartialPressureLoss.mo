within MetroscopeModelingLibrary.Common.PressureLosses;
model PartialPressureLoss "Singular pressure loss"
  extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;
  Modelica.SIunits.MassFlowRate Q(start=100) "Inlet Mass flow rate";
   MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP "Singular pressure loss";
equation
  deltaP = P_out - P_in;
  Q = Q_in;
  Q_in + Q_out = 0;
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
<p><b>Copyright &copy; </b>Metroscope</p>
<p>Metroscope Modeling Library</p>
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
end PartialPressureLoss;
