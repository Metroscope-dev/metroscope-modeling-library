within MetroscopeModelingLibrary.Utilities.Interfaces;
connector RealInput = input Real "'input Real' as connector" annotation (
  defaultComponentName="u",
  Icon(graphics={
    Polygon(
      lineColor={0,0,127},
      fillColor={0,0,127},
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
      lineColor={0,0,127},
      fillColor={0,0,127},
      fillPattern=FillPattern.Solid,
      points={{0,10},{20,0},{0,-10},{0,10}}),
    Text(
      textColor={0,0,127},
      extent={{0,-10},{0,10}},
      textString="%name",
        origin={-20,8.15782e-32},
        rotation=90)}),
  Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
