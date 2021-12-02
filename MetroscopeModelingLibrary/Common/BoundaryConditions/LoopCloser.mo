within MetroscopeModelingLibrary.Common.BoundaryConditions;
model LoopCloser
extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;
equation
  h_out = h_in;
  P_out = P_in;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Line(
          points={{-100,0},{98,0}},
          color={0,0,0},
          thickness=0.5)}),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Line(
          points={{-100,0},{98,0}},
          color={0,0,0},
          thickness=0.5), Ellipse(
          extent={{-28,28},{30,-30}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-20,22},{24,-24}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={244,125,35},
          fillPattern=FillPattern.None,
          textString="Q"),
        Line(
          points={{-20,-20},{20,20}},
          color={0,0,0},
          thickness=1)}),
    Documentation(info="<html>
<p><b>Copyright &copy; EDF 2002 - 2013</b> </p>
<p><b>ThermoSysPro Version 3.1</b> </p>
</html>",
   revisions="<html>
<u><p><b>Author</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Metroscope.tech</li>
</ul>
</html>
"));
end LoopCloser;
