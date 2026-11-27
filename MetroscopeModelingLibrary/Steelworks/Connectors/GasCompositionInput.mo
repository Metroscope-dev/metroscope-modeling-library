within MetroscopeModelingLibrary.Steelworks.Connectors;
connector GasCompositionInput "Steelworks gas composition as mole or mass fractions"
  input Real CO "Carbon monoxide";
  input Real CO2 "Carbon dioxide";
  input Real N2 "Nitrogen";
  input Real H2 "Hydrogen";
  input Real CH4 "Methane";
  input Real H2O "Water vapor";

  annotation (
  defaultComponentName="composition",
  Icon(graphics={Polygon(
        points={{0,20},{-20,-20},{20,-20},{0,20}},
        lineColor={213,170,255},
        lineThickness=0.5,
        fillColor={213,170,255},
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
        lineColor={213,170,255},
        lineThickness=0.5,
        fillColor={213,170,255},
        fillPattern=FillPattern.Solid,
        rotation=270)}));
end GasCompositionInput;
