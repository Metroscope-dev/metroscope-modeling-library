within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "Basic fluid transport brick for all components"
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.BaseClassIcon;
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Types;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // ------ Initialization parameters ------
  // Config
  parameter Types.Plant plant = Types.Plant.Nuclear;
  parameter Types.PressureLevel pressure_level = Types.PressureLevel.HP;
  parameter Types.Medium medium = Types.Medium.Water;
  parameter Types.Line line = Types.Line.Main;

  // Temperatures
  parameter Units.Temperature T_in_0 = Constants.T_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  parameter Units.Temperature T_out_0 = Constants.T_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  // Pressure
  parameter Units.Pressure P_in_0 = Constants.P_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  parameter Units.Pressure P_out_0 = P_in_0 + DP_0;
  parameter Units.DifferentialPressure DP_0 = Constants.DP_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  // Enthalpy
  parameter Units.SpecificEnthalpy h_in_0 = Constants.Q_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  parameter Units.SpecificEnthalpy h_out_0 = Constants.Q_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];
  // Density
  parameter Units.Density rho_0 = 998;
  // Mass flow rate
  parameter Units.PositiveMassFlowRate Q_0 = Constants.Q_0_values[Integer(plant), Integer(medium), Integer(pressure_level), Integer(line)];

  // ------ Input Quantities ------
  // Enthalpies
  Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
  Units.SpecificEnthalpy h_out(start=h_out_0) "Outlet specific enthalpy";
  // Mass flow rate
  Units.PositiveMassFlowRate Q(start=Q_0, nominal=Q_0);
  // Pressures
  Units.Pressure P_in(start=P_in_0, nominal=P_in_0) "Inlet Pressure";
  Units.Pressure P_out(start=P_out_0, nominal=P_out_0) "Outlet Pressure";
  // Mass fractions
  Units.MassFraction Xi[Medium.nXi] "Species mass fraction";

  // ------ Computed Quantities ------
  // Densities
  Units.Density rho_in(start=rho_0, nominal=rho_0) "Inlet density";
  Units.Density rho_out(start=rho_0, nominal=rho_0) "Outlet density";
  Units.Density rho(start=rho_0, nominal=rho_0) "Mean density";

  // Volumetric flow rates
  Units.PositiveVolumeFlowRate Qv_in(start=Q_0/rho_0, nominal=Q_0/rho_0) "Inlet volumetric flow rate";
  Units.NegativeVolumeFlowRate Qv_out(start=-Q_0/rho_0, nominal=Q_0/rho_0) "Outlet volumetric flow rate";
  Units.PositiveVolumeFlowRate Qv(start=Q_0/rho_0, nominal=Q_0/rho_0) "Mean volumetric flow rate";

  // Temperatures
  Units.Temperature T_in(start=T_in_0, nominal=T_in_0) "Fluid temperature";
  Units.Temperature T_out(start=T_out_0, nominal=T_out_0) "Fluid temperature";

  // ------ States ------
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  // ------ Conservation variables ------
  Units.DifferentialPressure DP(nominal=P_in_0, start=DP_0); // Pressure Loss
  Units.Power W(nominal=Q_0*h_in_0, start=Q_0*(h_out_0 - h_in_0)); // Heat Loss
  Units.DifferentialEnthalpy DH(start=h_out_0 - h_in_0);
  Units.DifferentialTemperature DT(start=T_out_0 - T_in_0);

  // ------ Connectors ------
  replaceable Partial.Connectors.FluidInlet C_in(
    Q(start=Q_0, nominal=Q_0),
    P(start=P_in_0, nominal=P_in_0), h_outflow(start=1e5, nominal=h_in_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  replaceable Partial.Connectors.FluidOutlet C_out(
    Q(start=Q_0, nominal=Q_0),
    P(start=P_out_0, nominal=P_out_0), h_outflow(start=h_out_0, nominal=h_out_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
equation
  assert(not (plant == Types.Plant.Nuclear and medium == Types.Medium.Fuel), "Fuel is not a valid medium for Nuclear plants, in component %name");
  assert(not (plant == Types.Plant.Nuclear and medium == Types.Medium.FlueGases), "FluesGases is not a valid medium for Nuclear plants, in component %name");
  // ------ Input Quantities ------
  // Enthalpies
  h_in = inStream(C_in.h_outflow);
  h_out = C_out.h_outflow;

  // Mass flow rate
  Q = C_in.Q;

  // Pressure
  P_in = C_in.P;
  P_out = C_out.P;

  // Mass Fractions
  Xi = inStream(C_in.Xi_outflow);

  // No flow reversal in stream connector
  C_in.h_outflow = 1e6; // Never used arbitrary value
  C_in.Xi_outflow = zeros(Medium.nXi); // No flow reversal

  // ------ States ------
  state_in = Medium.setState_phX(P_in, h_in, Xi);
  state_out = Medium.setState_phX(P_out, h_out, Xi);

  // ------ Computed Quantities ------
  // Temperatures
  T_in = Medium.temperature(state_in);
  T_out = Medium.temperature(state_out);

  // Densities
  rho_in = Medium.density(state_in);
  rho_out = Medium.density(state_out);
  rho = (rho_in + rho_out)/2;

  // Volumetric flow rates
  Qv_in = Q/rho_in;
  Qv_out = -Q/rho_out;
  Qv = (Qv_in - Qv_out)/2;

  // ------ Conservation equations ------
  P_out - P_in = DP;
  Q * (h_out - h_in) = W;
  h_out - h_in = DH;
  T_out - T_in = DT;
  C_in.Q + C_out.Q = 0;
  C_out.Xi_outflow = inStream(C_in.Xi_outflow);

  assert(Q > 0, "Wrong flow sign. Common causes : outlet connected as if it was inlet and vice versa, or Positive/NegativeMassflowrate misuse. Recall : inlet flow is positive, outlet is negatve", AssertionLevel.warning);
end FlowModel;
