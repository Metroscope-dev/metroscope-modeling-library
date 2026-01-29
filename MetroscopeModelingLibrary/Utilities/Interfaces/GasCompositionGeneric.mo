within MetroscopeModelingLibrary.Utilities.Interfaces;
connector GasCompositionGeneric "Real output connector used for vector of connectors"
  Real CH4(start=0.9) "Methane";
  Real C2H6(start=0.05) "Ethane";
  Real C3H8(start=0) "Propane";
  Real C4H10_n_butane(start=0) "n-Butane";
  Real N2(start=0.025) "Nitrogen";
  Real CO2(start=0.025) "Cardon dioxide";
                                                        annotation (
  defaultComponentName="y",
  Icon(                                  coordinateSystem(
      extent={{-40,-40},{40,40}},
      preserveAspectRatio=true,
      initialScale=0.2), graphics={
                 Polygon(
        points={{0,20},{-40,-60},{40,-60},{0,20}},
        lineColor={211,211,0},
        lineThickness=0.5,
        rotation=270,
          origin={20,0})}),
  Diagram(coordinateSystem(
      preserveAspectRatio=false,
      initialScale=0.2,
      extent={{-40,-40},{40,40}}),     graphics={
                 Polygon(
        points={{0,20},{-20,-20},{20,-20},{0,20}},
        lineColor={211,211,0},
        lineThickness=0.5,
        fillColor={211,211,0},
        fillPattern=FillPattern.Solid,
        rotation=270,
          origin={20,0})}),
  Documentation(info="<html>
<p>
Real output connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Routing.DeMultiplex\">DeMultiplex</a>,
and has therefore a different icon as RealOutput connector.
</p>
</html>"));
end GasCompositionGeneric;
