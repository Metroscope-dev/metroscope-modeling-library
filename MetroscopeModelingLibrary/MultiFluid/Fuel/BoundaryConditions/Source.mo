within MetroscopeModelingLibrary.MultiFluid.Fuel.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.MultiFluid.Fuel.Connectors.Outlet C_out,
                                                                                                                   redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));

    // Atomic mass
  constant Real amC=12.01115 "Carbon atomic mass";
  constant Real amH=1.00797 "Hydrogen atomic mass";
  constant Real amO=15.9994 "Oxygen atomic mass";
  constant Real amN=14.0067 "Nitrogen atomic mass";

  // Molar mass of species of interest
  Real amCH4 "CH4 molecular mass";
  Real amC2H6 "C2H6 molecular mass";
  Real amC3H8 "C3H8 molecular mass";
  Real amC4H10 "C4H10 molecular mass";
  Real amCO2 "CO2 molecular mass";
  Real amN2 "H2O molecular mass";

  // Fuel composition
  Utilities.Units.MassFraction X_CH4(start=0.848);
  Utilities.Units.MassFraction X_C2H6(start=0.083);
  Utilities.Units.MassFraction X_C3H8(start=0.0126);
  Utilities.Units.MassFraction X_C4H10_n_butane(start=0.00668);
  Utilities.Units.MassFraction X_N2(start=0.024);
  Utilities.Units.MassFraction X_CO2(start=0.025);

  // Mole fractions
  Real X_molar_CH4(start=0.92);
  Real X_molar_C2H6(start=0.048);
  Real X_molar_C3H8(start=0.005);
  Real X_molar_C4H10_n_butane(start=0.002);
  Real X_molar_N2(start=0.015);
  Real X_molar_CO2(start=0.01);

  // Mean molecular mass
  Real mean_molecular_mass(start=17);

equation

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

  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end Source;
