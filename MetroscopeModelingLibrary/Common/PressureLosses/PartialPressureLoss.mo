within MetroscopeModelingLibrary.Common.PressureLosses;
partial model PartialPressureLoss "Singular pressure loss"
  extends MetroscopeModelingLibrary.Common.Partial.FlowModel;
  Modelica.Units.SI.MassFlowRate Q(start=100) "Inlet Mass flow rate";
  Common.Units.DifferentialPressure deltaP "Singular pressure loss";
equation
  deltaP = P_out - P_in;
  Q = Q_in;
  //DM = 0;
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
