within MetroscopeModelingLibrary.Utilities.Interfaces;
connector BoundaryCondition
                    = input Real "'input Real' as connector"  annotation (
  Dialog,
  signalLogging=true,
  defaultComponentName="u",
  Icon(graphics={
  Text(
      extent={{-100,-160},{102,-200}},
      textColor={0,0,0},
      textString=DynamicSelect("",String(start))),
    Polygon(
      lineColor={28,108,200},
      fillColor={238,46,47},
      fillPattern=FillPattern.Solid,
      points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
    coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
      preserveAspectRatio=true,
      initialScale=0.2)),
  Diagram(
    coordinateSystem(preserveAspectRatio=true,
      initialScale=0.2,
      extent={{-20,-20},{20,20}}),
      graphics={
    Polygon(
      lineColor={238,46,47},
      fillColor={238,46,47},
      fillPattern=FillPattern.Solid,
      points={{0,10},{20,0},{0,-10},{0,10}}),
    Text(
      textColor={238,46,47},
      extent={{-80,-20},{0,20}},
      textString="%name",
        origin={-20,40},
        rotation=90)}),
  Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
