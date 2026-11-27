within MetroscopeModelingLibrary.Steelworks.BoundaryConditions;
model Source "Source of CO, CO2, N2, H2, CH4 and H2O"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasMedium;
  extends Partial.BoundaryConditions.FluidSource(
    redeclare MetroscopeModelingLibrary.Steelworks.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium,
    Xi_out(start=SteelworksGasMedium.reference_X)) annotation (IconMap(primitivesVisible=false));

  Utilities.Units.MassFraction X_CO;
  Utilities.Units.MassFraction X_CO2;
  Utilities.Units.MassFraction X_N2;
  Utilities.Units.MassFraction X_H2;
  Utilities.Units.MassFraction X_CH4;
  Utilities.Units.MassFraction X_H2O;

  Modelica.Units.SI.MoleFraction X_molar_CO(start=X_molar_0[1]);
  Modelica.Units.SI.MoleFraction X_molar_CO2(start=X_molar_0[2]);
  Modelica.Units.SI.MoleFraction X_molar_N2(start=X_molar_0[3]);
  Modelica.Units.SI.MoleFraction X_molar_H2(start=X_molar_0[4]);
  Modelica.Units.SI.MoleFraction X_molar_CH4(start=X_molar_0[5]);
  Modelica.Units.SI.MoleFraction X_molar_H2O(start=X_molar_0[6]);

  Modelica.Units.SI.MolarMass mean_molecular_mass(start=X_molar_0*Medium.MMX);

protected
  final parameter Modelica.Units.SI.MoleFraction X_molar_0[Medium.nX] =
    Medium.massToMoleFractions(Medium.reference_X, Medium.MMX);
  Modelica.Units.SI.MoleFraction X_molar[Medium.nX];

equation
  Xi_out = {X_CO, X_CO2, X_N2, X_H2, X_CH4, X_H2O};
  X_molar = {X_molar_CO, X_molar_CO2, X_molar_N2, X_molar_H2, X_molar_CH4, X_molar_H2O};

  // The caller can prescribe either set of fractions. Both sums are equal.
  sum(X_molar)*mean_molecular_mass = X_molar*Medium.MMX;
  Xi_out*mean_molecular_mass = X_molar .* Medium.MMX;

  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}), Diagram(coordinateSystem(initialScale=0.2)));
end Source;
