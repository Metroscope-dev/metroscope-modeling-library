within MetroscopeModelingLibrary.Utilities.Interfaces;
connector GasCompositionInput "Gas composition. Choose composition unit in the chromatograph component"

  input Real CH4(start=0.9) "Methane";
  input Real C2H6(start=0.05) "Ethane";
  input Real C3H8(start=0) "Propane";
  input Real C4H10_n_butane(start=0) "n-Butane";
  input Real N2(start=0.025) "Nitrogen";
  input Real CO2(start=0.025) "Cardon dioxide";
                                                       annotation (
  defaultComponentName="composition",
  Icon(graphics={Polygon(
        points={{0,20},{-20,-20},{20,-20},{0,20}},
        lineColor={211,211,0},
        lineThickness=0.5,
        fillColor={211,211,0},
        fillPattern=FillPattern.Solid,
        rotation=270)},                  coordinateSystem(
      extent={{-20,-20},{20,20}},
      preserveAspectRatio=false,
      initialScale=0.2)),
  Diagram(coordinateSystem(
      preserveAspectRatio=false,
      extent={{-20,-20},{20,20}},
      initialScale=0.2),               graphics={Text(
        extent={{0,12.5},{0,-12.5}},
        textColor={0,0,127},
        origin={-10,0},
        rotation=90,
          fontSize=2,
          textString="%name"),
                      Polygon(
        points={{0,20},{-10,0},{10,0},{0,20}},
        lineColor={211,211,0},
        lineThickness=0.5,
        fillColor={211,211,0},
        fillPattern=FillPattern.Solid,
        rotation=270)}),
  Documentation(info="<html>
<p>
Real input connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Interfaces.PartialRealMISO\">PartialRealMISO</a>,
and has therefore a different icon as RealInput connector.
</p>
</html>"));
end GasCompositionInput;
