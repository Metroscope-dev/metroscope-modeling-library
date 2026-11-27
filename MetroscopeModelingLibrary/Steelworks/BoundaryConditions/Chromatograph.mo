within MetroscopeModelingLibrary.Steelworks.BoundaryConditions;
model Chromatograph "CO, CO2, N2, H2, CH4 and H2O"
  extends Source(
    Xi_out=if signal_unit == "%mol" then
      Medium.moleToMassFractions(compositionFractions, Medium.MMX) else compositionFractions)
    annotation (IconMap(primitivesVisible=false));

  parameter String signal_unit = "%mol" "Composition basis. Signals are fractions from 0 to 1"
    annotation (choices(choice="%mol", choice="%mass"));

  Steelworks.Connectors.GasCompositionInput composition(
    CO(start=composition_start[1]),
    CO2(start=composition_start[2]),
    N2(start=composition_start[3]),
    H2(start=composition_start[4]),
    CH4(start=composition_start[5]),
    H2O(start=composition_start[6])) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-58,0}), iconTransformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-76,0})));

protected
  final parameter Real composition_start[Medium.nX] =
    if signal_unit == "%mol" then X_molar_0 else Medium.reference_X;
  Real compositionFractions[Medium.nX];

equation
  compositionFractions = {composition.CO, composition.CO2, composition.N2,
                          composition.H2, composition.CH4, composition.H2O};

  annotation (Icon(coordinateSystem(initialScale=0.2), graphics={
        Ellipse(
          extent={{-70,60},{50,-60}},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}),        Diagram(coordinateSystem(initialScale=0.2)));
end Chromatograph;
