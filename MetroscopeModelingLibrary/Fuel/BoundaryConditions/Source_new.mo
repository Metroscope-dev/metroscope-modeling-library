within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Source_new
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FuelSourceIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));

  // Atomic Mass
  constant Real amC=12.01115 "Carbon atomic mass";
  constant Real amH=1.00797 "Hydrogen atomic mass";
  constant Real amO=15.9994 "Oxygen atomic mass";
  constant Real amN=14.0067 "Nitrogen atomic mass";

  // Molar mass
  Real amCH4 "CH4 molecular mass";
  Real amC2H6 "C2H6 molecular mass";
  Real amC3H8 "C3H8 molecular mass";
  Real amC4H10 "C4H10 molecular mass";
  Real amCO2 "CO2 molecular mass";
  Real amN2 "H2O molecular mass";

  // Fuel composition
  Units.MassFraction X_fuel_CH4(start=0.92);
  Units.MassFraction X_fuel_C2H6(start=0.048);
  Units.MassFraction X_fuel_C3H8(start=0.005);
  Units.MassFraction X_fuel_C4H10_n_butane(start=0.002);
  Units.MassFraction X_fuel_N2(start=0.015);
  Units.MassFraction X_fuel_CO2(start=0.01);

  // Mole fractions
  Real X_molar_fuel_CH4(start=0.92);
  Real X_molar_fuel_C2H6(start=0.048);
  Real X_molar_fuel_C3H8(start=0.005);
  Real X_molar_fuel_C4H10_n_butane(start=0.002);
  Real X_molar_fuel_N2(start=0.015);
  Real X_molar_fuel_CO2(start=0.01);

  // Mean molecular mass
  Real mean_molecular_mass(start=16);

  // Lower heating value
    output Real LHV "J/kg";
    // Specify if the LHV is an "external" input or should be "calculated" form the composition
    parameter String LHV_source = "external";
    input Real LHV_given(start=47e6);

equation

  // Molar mass of species of interest
  amCH4 = amC + 4*amH;
  amC2H6 = 2*amC + 6*amH;
  amC3H8 = 3*amC + 8*amH;
  amC4H10 = 4*amC + 10*amH;
  amN2 = 2*amN;
  amCO2 = amC + 2*amO;


  // Composition mass fraction
  X_fuel_CH4 = Xi_out[1]; // methane
  X_fuel_C2H6 = Xi_out[2]; // ethane
  X_fuel_C3H8=  Xi_out[3]; // propane
  X_fuel_C4H10_n_butane=  Xi_out[4]; //butane
  X_fuel_N2=  Xi_out[5]; // nitrogen
  X_fuel_CO2=  Xi_out[6]; // carbon dioxyde

  // Mean Molecular Mass
  mean_molecular_mass = 1/(X_fuel_CH4/amCH4 + X_fuel_C2H6/amC2H6 + X_fuel_C3H8/amC3H8 + X_fuel_C4H10_n_butane/amC4H10 + X_fuel_N2/amN2 + X_fuel_CO2/amCO2);

  // Mass and mole fraction relation
  X_molar_fuel_CH4 = X_fuel_CH4/amCH4 * mean_molecular_mass;
  X_molar_fuel_C2H6 = X_fuel_C2H6/amC2H6 * mean_molecular_mass;
  X_molar_fuel_C3H8 = X_fuel_C3H8/amC3H8 * mean_molecular_mass;
  X_molar_fuel_C4H10_n_butane = X_fuel_C4H10_n_butane/amC4H10 * mean_molecular_mass;
  X_molar_fuel_N2 = X_fuel_N2/amN2 * mean_molecular_mass;
  X_molar_fuel_CO2 = X_fuel_CO2/amCO2 * mean_molecular_mass;

  // LHV calculation, based on ISO6976
  if LHV_source == "calculated" then
  LHV = (X_molar_fuel_CH4*802.69 + X_molar_fuel_C2H6*1428.83 + X_molar_fuel_C3H8*2043.35 + X_molar_fuel_C4H10_n_butane*2657.58)*1e6/mean_molecular_mass;
  else
    LHV = LHV_given;
  end if;





end Source_new;
