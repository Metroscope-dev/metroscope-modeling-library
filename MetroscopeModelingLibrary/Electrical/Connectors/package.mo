within MetroscopeModelingLibrary.Electrical;
package Connectors
  connector C_power

    flow Modelica.Units.SI.Power W;

    annotation (
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
        extent={{-100.0,-100.0},{100.0,100.0}}),
        graphics={
      Polygon(
        lineColor={0,0,127},
        fillColor={0,0,127},
        fillPattern=FillPattern.Solid,
        points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}})}),
    Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));

  end C_power;
end Connectors;
