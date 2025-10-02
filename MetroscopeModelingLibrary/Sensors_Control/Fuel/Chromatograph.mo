within MetroscopeModelingLibrary.Sensors_Control.Fuel;
model Chromatograph
  Utilities.Interfaces.GenericReal X_CO2 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={20,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={84,36})));
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

  extends Partial.Sensors.BaseSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;

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

  // Mole fractions
  Real X_molar_CH4(start=0.92);
  Real X_molar_C2H6(start=0.048);
  Real X_molar_C3H8(start=0.005);
  Real X_molar_C4H10_n_butane(start=0.002);
  Real X_molar_N2(start=0.015);
  Real X_molar_CO2(start=0.01);

  // Mean molecular mass
  Real mean_molecular_mass(start=17);

  Utilities.Interfaces.GenericReal X_C2H6 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={-60,76})));
  Utilities.Interfaces.GenericReal X_C3H8 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={-22,104})));
  Utilities.Interfaces.GenericReal X_C4H10_n_butane annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={20,104})));
  Utilities.Interfaces.GenericReal X_N2                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={60,78})));
equation

  // Molar mass of species of interest
public
  Utilities.Interfaces.GenericReal X_CH4 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,40}), iconTransformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={-84,38})));
equation
  amCH4 = amC + 4*amH;
  amC2H6 = 2*amC + 6*amH;
  amC3H8 = 3*amC + 8*amH;
  amC4H10 = 4*amC + 10*amH;
  amN2 = 2*amN;
  amCO2 = amC + 2*amO;

  // Composition mass fraction
  X_CH4/100 = Xi[1]; // methane
  X_C2H6/100 = Xi[2]; // ethane
  X_C3H8/100 = Xi[3]; // propane
  X_C4H10_n_butane/100 = Xi[4]; // butane
  X_N2/100 = Xi[5]; // nitrogen
  X_CO2/100 = Xi[6]; // carbon dioxyde

  // Mean Molecular Mass: this gives the correct results only if the molar fraction is given as an input, if the mass fraction is given, this quantity is useless
  mean_molecular_mass = X_molar_CH4*amCH4 + X_molar_C2H6*amC2H6 + X_molar_C3H8*amC3H8 + X_molar_C4H10_n_butane*amC4H10 + X_molar_N2*amN2 + X_molar_CO2*amCO2;

  // Mass and mole fraction relation
  X_molar_CH4 = X_CH4/amCH4 * mean_molecular_mass;
  X_molar_C2H6 = X_C2H6/amC2H6 * mean_molecular_mass;
  X_molar_C3H8 = X_C3H8/amC3H8 * mean_molecular_mass;
  X_molar_C4H10_n_butane = X_C4H10_n_butane/amC4H10 * mean_molecular_mass;
  X_molar_N2 = X_N2/amN2 * mean_molecular_mass;
  X_molar_CO2 = X_CO2/amCO2 * mean_molecular_mass;

  annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
                   graphics={Text(
          extent={{-60,60},{60,-60}},
          textColor={0,0,0},
          textString="X")}),    Diagram(coordinateSystem(extent={{-100,-100},{100,
            100}})));
end Chromatograph;
