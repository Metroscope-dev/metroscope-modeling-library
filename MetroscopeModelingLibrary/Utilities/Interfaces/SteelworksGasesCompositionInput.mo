within MetroscopeModelingLibrary.Utilities.Interfaces;
connector SteelworksGasesCompositionInput "Steelworks gas composition as mole or mass fractions"
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;

  input Real N2(start=Medium.referenceMoleFractions[1]) "Nitrogen";
  input Real CH4(start=Medium.referenceMoleFractions[2]) "Methane";
  input Real CO2(start=Medium.referenceMoleFractions[3]) "Carbon dioxide";
  input Real H2(start=Medium.referenceMoleFractions[4]) "Hydrogen";
  input Real CO(start=Medium.referenceMoleFractions[5]) "Carbon monoxide";
  input Real O2(start=Medium.referenceMoleFractions[6]) "Oxygen";
  input Real H2O(start=Medium.referenceMoleFractions[7]) "Water vapor";

  annotation (
    Documentation(info="<html><p>Choose the basis in the chromatograph. Start values are the normalized reference mole fractions.</p></html>"),
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
end SteelworksGasesCompositionInput;
