within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Chromatograph
                   "CH4, C2H6, C3H8, n-C4H10, N2, CO2"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));

    // Atomic mass
  constant Real amC=12.01115 "Carbon atomic mass";
  constant Real amH=1.00797 "Hydrogen atomic mass";
  constant Real amO=15.9994 "Oxygen atomic mass";
  constant Real amN=14.0067 "Nitrogen atomic mass";

  // Signal unit
  parameter String signal_unit = "%mol" "Choose the unit of the composition signals" annotation (choices(choice="%mol", choice="%mass"));

  // Molar mass of species of interest
  Real amCH4 "CH4 molecular mass";
  Real amC2H6 "C2H6 molecular mass";
  Real amC3H8 "C3H8 molecular mass";
  Real amC4H10 "C4H10 molecular mass";
  Real amCO2 "CO2 molecular mass";
  Real amN2 "H2O molecular mass";

  // Fuel composition
  Utilities.Units.MassFraction X_CH4;
  Utilities.Units.MassFraction X_C2H6;
  Utilities.Units.MassFraction X_C3H8;
  Utilities.Units.MassFraction X_C4H10_n_butane;
  Utilities.Units.MassFraction X_N2;
  Utilities.Units.MassFraction X_CO2;

  // Mole fractions
  Real X_molar_CH4;
  Real X_molar_C2H6;
  Real X_molar_C3H8;
  Real X_molar_C4H10_n_butane;
  Real X_molar_N2;
  Real X_molar_CO2;

  // Mean molecular mass
  Real mean_molecular_mass(start=17);

  Utilities.Interfaces.GasCompositionGeneric composition "CH4, C2H6, C3H8, n-C4H10, N2, CO2" annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-58,0}), iconTransformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-76,0})));
equation

  // Connect to chromatograph
  if signal_unit == "%mol" then
    X_molar_CH4 = composition.CH4;
    X_molar_C2H6 = composition.C2H6;
    X_molar_C3H8 = composition.C3H8;
    X_molar_C4H10_n_butane = composition.C4H10_n_butane;
    X_molar_N2 = composition.N2;
    X_molar_CO2 = composition.CO2;
  else
    X_CH4 = composition.CH4;
    X_C2H6 = composition.C2H6;
    X_C3H8 = composition.C3H8;
    X_C4H10_n_butane = composition.C4H10_n_butane;
    X_N2 = composition.N2;
    X_CO2 = composition.CO2;
  end if;

  // Molar mass of species of interest
  amCH4 = amC + 4*amH;
  amC2H6 = 2*amC + 6*amH;
  amC3H8 = 3*amC + 8*amH;
  amC4H10 = 4*amC + 10*amH;
  amN2 = 2*amN;
  amCO2 = amC + 2*amO;

  // Composition mass fraction
  X_CH4 = Xi_out[1]; // methane
  X_C2H6 = Xi_out[2]; // ethane
  X_C3H8 = Xi_out[3]; // propane
  X_C4H10_n_butane = Xi_out[4]; // butane
  X_N2 = Xi_out[5]; // nitrogen
  X_CO2 = Xi_out[6]; // carbon dioxyde

  // Mean Molecular Mass: this gives the correct results only if the molar fraction is given as an input, if the mass fraction is given, this quantity is useless
  mean_molecular_mass = X_molar_CH4*amCH4 + X_molar_C2H6*amC2H6 + X_molar_C3H8*amC3H8 + X_molar_C4H10_n_butane*amC4H10 + X_molar_N2*amN2 + X_molar_CO2*amCO2;

  // Mass and mole fraction relation
  X_molar_CH4 = X_CH4/amCH4 * mean_molecular_mass;
  X_molar_C2H6 = X_C2H6/amC2H6 * mean_molecular_mass;
  X_molar_C3H8 = X_C3H8/amC3H8 * mean_molecular_mass;
  X_molar_C4H10_n_butane = X_C4H10_n_butane/amC4H10 * mean_molecular_mass;
  X_molar_N2 = X_N2/amN2 * mean_molecular_mass;
  X_molar_CO2 = X_CO2/amCO2 * mean_molecular_mass;

  annotation (Icon(coordinateSystem(initialScale=0.2), graphics={
        Ellipse(
          extent={{-70,60},{50,-60}},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}),        Diagram(coordinateSystem(initialScale=0.2)));
end Chromatograph;
