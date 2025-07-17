within MetroscopeModelingLibrary.Utilities.Interfaces;
connector Observable = output Real "'output Real' as connector" annotation (
  defaultComponentName="y",
  Icon(
    coordinateSystem(preserveAspectRatio=true,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
    Polygon(
      lineColor={0,0,127},
      fillColor={255,255,255},
      fillPattern=FillPattern.Solid,
      points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
  Diagram(
    coordinateSystem(preserveAspectRatio=true,
      extent={{-20,-20},{20,20}},
      initialScale=0.2),
      graphics={
    Text(
      textColor={0,0,127},
      extent={{-80,-20},{0,20}},
      textString="%name",
        origin={-20,40},
        rotation=90),
    Polygon(
      lineColor={0,0,127},
      fillColor={255,255,255},
      fillPattern=FillPattern.Solid,
      points={{0,10},{20,0},{0,-10},{0,10}})}),
  Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));
