within MetroscopeModelingLibrary.Utilities.Interfaces;
connector GenericReal = Real "generic connector"         annotation (
  defaultComponentName="u",
  Icon(graphics={
    Polygon(
      lineColor={0,0,127},
      fillColor={135,135,135},
      fillPattern=FillPattern.Solid,
      points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}}),
    Text(
      textColor={95,95,95},
      extent={{140,100},{-140,160}},
        textString="%name")},
    coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
      preserveAspectRatio=true,
      initialScale=0.2)),
  Diagram(
    coordinateSystem(preserveAspectRatio=true,
      initialScale=0.2,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
    Polygon(
      lineColor={0,0,127},
      fillColor={0,0,127},
      fillPattern=FillPattern.Solid,
      points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
    Text(
      textColor={0,0,127},
      extent={{140,60},{-40,100}},
      textString="%name")}),
  Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));
