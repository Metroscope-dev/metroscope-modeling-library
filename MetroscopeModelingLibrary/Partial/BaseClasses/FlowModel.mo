within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model FlowModel "Basic fluid transport brick for all components"
  extends MetroscopeModelingLibrary.Icons.BaseClasses.BaseClassIcon;
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // ------ Initialization parameters ------
  // Temperatures
  parameter Units.Temperature T_in_0 = 300;
  parameter Units.Temperature T_out_0 = 300;
  // Pressure
  parameter Units.Pressure P_in_0 = 1e5;
  parameter Units.Pressure P_out_0 = 1e5;
  parameter Units.DifferentialPressure DP_0 = P_out_0 - P_in_0;
  // Enthalpy
  parameter Units.SpecificEnthalpy h_in_0 = 5e5;
  parameter Units.SpecificEnthalpy h_out_0 = 5e5;
  // Mass flow rate
  parameter Units.PositiveMassFlowRate Q_0 = 1000 "Inlet Mass flow rate";

  // ------ Input Quantities ------
  // Enthalpies
  Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
  Units.SpecificEnthalpy h_out(start=h_out_0) "Outlet specific enthalpy";
  // Mass flow rate
  Units.PositiveMassFlowRate Q(start=Q_0) "Inlet Mass flow rate";
  // Pressures
  Units.Pressure P_in(start=P_in_0) "Inlet Pressure";
  Units.Pressure P_out(start=P_out_0) "Outlet Pressure";
  // Mass fractions
  Units.MassFraction Xi[Medium.nXi] "Species mass fraction";

  // ------ Computed Quantities ------
  // Densities
  Units.Density rho_in "Inlet density";
  Units.Density rho_out "Outlet density";
  Units.Density rho "Mean density";

  // Volumetric flow rates
  Units.PositiveVolumeFlowRate Qv_in "Inlet volumetric flow rate";
  Units.NegativeVolumeFlowRate Qv_out "Outlet volumetric flow rate";

  // Temperatures
  Units.Temperature T_in(start=T_in_0) "Fluid temperature";
  Units.Temperature T_out(start=T_out_0) "Fluid temperature";

  // ------ States ------
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  // ------ Conservation variables ------
  Units.DifferentialPressure DP(nominal=DP_0, start=DP_0); // Pressure Loss
  Units.Power W(nominal=0, start=0); // Heat Loss

  // ------ Connectors ------
  replaceable Partial.Connectors.FluidInlet C_in(
    Q(start=Q_0),
    P(start=P_in_0, nominal=P_in_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  replaceable Partial.Connectors.FluidOutlet C_out(
    Q(start=-Q_0),
    P(start=P_out_0, nominal=P_out_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
equation
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
  Xi = C_out.Xi_outflow;

  // No flow reversal in stream connector
  C_in.h_outflow = 0; // Never used arbitrary value
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

  // ------ Conservation equations ------
  P_out - P_in = DP;
  Q * (h_out - h_in) = W;
  C_in.Q + C_out.Q = 0;

end FlowModel;
