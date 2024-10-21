within MetroscopeModelingLibrary.MultiFluid.Machines;
model CombustionChamberwithRefMoistAir
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants.*;
  import MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet;
  import MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  // Media flows
  Units.PositiveMassFlowRate Q_air;
  Units.PositiveMassFlowRate Q_fuel;
  Units.PositiveMassFlowRate Q_exhaust;

  // Performance parameters
  Inputs.InputFrictionCoefficient Kfr(start=0);
  Inputs.InputReal eta(start=0.99457); // Efficiency from performance document of GE

  // Power released by the combustion
  Inputs.InputPower Wth;

  // Enthalpies at each connector
  Units.SpecificEnthalpy h_in_fuel;
  Units.SpecificEnthalpy h_exhaust;

  // Air intake composition (RefMoistAir-specific)
   Real relative_humidity;  // Relative humidity for moist air
   Units.SpecificEnthalpy h_in_air;  // Specific enthalpy for moist air

  // Exhaust composition
  Units.MassFraction X_out_N2(start=0.73);
  Units.MassFraction X_out_O2(start=0.12);
  Units.MassFraction X_out_H2O(start=0.10);
  Units.MassFraction X_out_CO2(start=0.05);
  Units.MassFraction X_out_SO2(start=0.0);

  // Fuel composition
  Units.MassFraction X_fuel_CH4(start=0.92);
  Units.MassFraction X_fuel_C2H6(start=0.048);
  Units.MassFraction X_fuel_C3H8(start=0.005);
  Units.MassFraction X_fuel_C4H10_n_butane(start=0.002);
  Units.MassFraction X_fuel_N2(start=0.015);
  Units.MassFraction X_fuel_CO2(start=0.01);

  Units.MassFraction X_fuel_C(start=0.8) "C mass fraction in the fuel";
  Units.MassFraction X_fuel_H(start=0.2) "H mass fraction in the fuel";
  Units.MassFraction X_fuel_O(start=0) "O mass fraction in the fuel";

  // Heating values
  Units.SpecificEnthalpy HHV = (hhv_mass_CH4*X_fuel_CH4 + hhv_mass_C2H6*X_fuel_C2H6 + hhv_mass_C3H8*X_fuel_C3H8 + hhv_mass_C4H10*X_fuel_C4H10_n_butane)*1e6 "J/kg";
  Units.SpecificEnthalpy LHV = HHV - 2202.92069 * m_H * (4*X_fuel_CH4/m_CH4 + 6*X_fuel_C2H6/m_C2H6 + 8*X_fuel_C3H8/m_C3H8 + 10*X_fuel_C4H10_n_butane/m_C4H10) * 1e4 "J/kg";

  // Initialization parameters
  parameter Units.SpecificEnthalpy h_in_air_0 = 5e5;

  // Inlets and outlets
  RefMoistAir.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Connectors.Outlet outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Fuel.Connectors.Inlet inlet1 annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  Fuel.BoundaryConditions.Sink sink_fuel annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-22})));
  FlueGases.BoundaryConditions.Source source_exhaust annotation (Placement(transformation(extent={{12,-10},{32,10}})));
  RefMoistAir.BoundaryConditions.Sink sink_air(h_in(start=h_in_air_0)) annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  FlueGases.Pipes.Pipe pressure_loss annotation (Placement(transformation(extent={{46,-10},{66,10}})));

equation

  // Definitions
  Q_air = sink_air.Q_in;
  Q_fuel = sink_fuel.Q_in;
  Q_exhaust = -source_exhaust.Q_out;

  h_in_air = sink_air.h_in;
  relative_humidity = sink_air.relative_humidity;  // Relative humidity for moist air inlet

  h_in_fuel = sink_fuel.h_in;
  h_exhaust = source_exhaust.h_out;

  X_fuel_CH4 = sink_fuel.Xi_in[1]; // Methane
  X_fuel_C2H6 = sink_fuel.Xi_in[2]; // Ethane
  X_fuel_C3H8 = sink_fuel.Xi_in[3]; // Propane
  X_fuel_C4H10_n_butane = sink_fuel.Xi_in[4]; // Butane
  X_fuel_N2 = sink_fuel.Xi_in[5]; // Nitrogen
  X_fuel_CO2 = sink_fuel.Xi_in[6]; // Carbon dioxide

  // Final quantities for exhaust gases
  X_out_N2 = source_exhaust.Xi_out[1];
  X_out_O2 = source_exhaust.Xi_out[2];
  X_out_H2O = source_exhaust.Xi_out[3];
  X_out_CO2 = source_exhaust.Xi_out[4];
  X_out_SO2 = source_exhaust.Xi_out[5];

  // Mass balance
  Q_exhaust = Q_air + Q_fuel;

  // Mechanical balance
  sink_air.P_in - source_exhaust.P_out = 0;
  pressure_loss.delta_z = 0;
  pressure_loss.Kfr = Kfr;

  // Energy balance
  Wth = eta * Q_fuel * LHV;
  Q_exhaust * h_exhaust = Q_air * h_in_air + Q_fuel * h_in_fuel + Wth;

  // Chemical balance for fuel
  X_fuel_C = m_C * (X_fuel_CH4/m_CH4 + 2*X_fuel_C2H6/m_C2H6 + 3*X_fuel_C3H8/m_C3H8 + 4*X_fuel_C4H10_n_butane/m_C4H10 + X_fuel_CO2/m_CO2);
  X_fuel_H = m_H * (4*X_fuel_CH4/m_CH4 + 6*X_fuel_C2H6/m_C2H6 + 8*X_fuel_C3H8/m_C3H8 + 10*X_fuel_C4H10_n_butane/m_C4H10);
  X_fuel_O = 2 * m_O * X_fuel_CO2/m_CO2;

  // Mass balance for all species
  - Q_exhaust*X_out_N2 + Q_air*0.78*(1 - sink_air.Xi_in[1]) + Q_fuel*X_fuel_N2 = 0; // N2 balance
  - Q_exhaust * X_out_O2  + Q_air * 0.21  + Q_fuel * X_fuel_O * 0.5 = Q_fuel * m_O * (2 * X_fuel_C/m_C + 0.5 * X_fuel_H/m_H); // O2 balance
  - Q_exhaust * X_out_H2O + Q_air * 0.01 = -Q_fuel * (0.5 * X_fuel_H/m_H) * m_H2O; // H2O balance
  - Q_exhaust * X_out_CO2 + Q_air * 0 = -Q_fuel * X_fuel_C * m_CO2/m_C; // CO2 balance
  - Q_exhaust * X_out_SO2 + Q_air * 0 = 0; // No sulfur in fuel

  connect(sink_air.C_in, inlet) annotation (Line(points={{-27,0},{-100,0}}, color={95,95,95}));
  connect(sink_fuel.C_in, inlet1) annotation (Line(points={{-2.77556e-16,-27},{-2.77556e-16,-63.5},{0,-63.5},{0,-100}}, color={213,213,0}));
  connect(source_exhaust.C_out, pressure_loss.C_in) annotation (Line(points={{27,0},{46,0}}, color={95,95,95}));
  connect(pressure_loss.C_out, outlet) annotation (Line(points={{66,0},{100,0}}, color={95,95,95}));
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Window(
      x=0.03,
      y=0.02,
      width=0.95,
      height=0.95),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{100,100},{100,-100},{-100,-100},{-100,100},{100,100}},
          lineColor={0,0,255},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-32,62},{-18.916,58.8281},{34,46},{-10,30},{46,18},{-6,0},{36,-16},{-10,-32},{42,-44},{-20.4375,-57.5},{-32,-60},{-32,-46},{-32,45.5586},{-32,62}},
          lineColor={158,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier)}));
end CombustionChamberwithRefMoistAir;
