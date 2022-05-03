within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model PartialTransportModel "Basic fluid transport brick for all components"

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
  // ------ Input Quantities ------
  // Enthalpies
  Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
  Units.SpecificEnthalpy h_out(start=h_out_0) "Outlet specific enthalpy";
  Units.SpecificEnthalpy hm "Average specific enthalpy";
  // Mass flow rate
  Units.PositiveMassFlowRate Q_in "Inlet Mass flow rate";
  Units.NegativeMassFlowRate Q_out "Outlet Mass flow rate";
  Units.PositiveMassFlowRate Qm "Mean Mass flow rate";
  // Pressures
  Units.Pressure P_in(start=P_in_0) "Inlet Pressure";
  Units.Pressure P_out(start=P_out_0) "Outlet Pressure";
  Units.Pressure Pm(start=(P_in_0 + P_out_0)/2) "Average fluid pressure";
  // Mass fractions
  Units.MassFraction Xi_in[Medium.nXi] "Inlet species mass fraction";
  Units.MassFraction Xi_out[Medium.nXi] "Outlet species mass fraction";
  Units.MassFraction Xim[Medium.nXi] "Outlet species mass fraction";
  //extends PartialTransportXi(redeclare package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium);

  // ------ Computed Quantities ------
  // Densities
  Units.Density rho_in "Inlet Fluid density";
  Units.Density rho_out "Outlet Fluid density";
  Units.Density rhom "Average Fluid density";

  // Volumic flow rates
  Units.PositiveVolumeFlowRate Qv_in "inlet volume flow rate";
  Units.NegativeVolumeFlowRate Qv_out "outlet volume flow rate";
  Units.PositiveVolumeFlowRate Qvm "Mean volume flow rate";

  // Temperatures
  Units.Temperature T_in(start=T_in_0) "Fluid temperature";
  Units.Temperature T_out(start=T_out_0) "Fluid temperature";
  Units.Temperature Tm(start=(T_in_0 + T_out_0)/2) "Fluid temperature";

  // ------ States ------
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  // ------ Conservation variables ------
  Units.MassFlowRate DM(nominal=0); // Mass Loss
  Units.DifferentialPressure DP(nominal=DP_0, start=DP_0); // Pressure Loss
  Units.Power W(nominal=0, start=0); // Heat Loss
  Units.MassFraction DXi[Medium.nXi] "species mass fraction variation in component";

  // ------ Connectors ------
  replaceable Partial.Connectors.FluidInlet C_in(
    Q,
    P(start=P_in_0, nominal=P_in_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  replaceable Partial.Connectors.FluidOutlet C_out(
    Q,
    P(start=P_out_0, nominal=P_out_0),
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
equation
  // ------ Input Quantities ------
  // Enthalpies
  h_in = inStream(C_in.h_outflow);
  h_out = C_out.h_outflow;
  hm = (h_in + h_out) / 2;

  // Mass flow rate
  Q_in = C_in.Q;
  Q_out = C_out.Q;
  Qm = (Q_in-Q_out)/2; // Q_in > 0 if the flow comes in, and Q_out < 0 if it goes out

  // Pressure
  P_in = C_in.P;
  P_out = C_out.P;
  Pm = (P_in + P_out)/2;

  // Mass Fractions
  Xi_in = inStream(C_in.Xi_outflow);
  Xi_out = C_out.Xi_outflow;
  Xim = (Xi_in + Xi_out)/2;

  // No flow reversal in stream connector
  C_in.h_outflow = 0; // Never used arbitrary value
  C_in.Xi_outflow = zeros(Medium.nXi); // No flow reversal

  // ------ States ------
  state_in = Medium.setState_phX(P_in, h_in, Xi_in);
  state_out = Medium.setState_phX(P_out, h_out, Xi_out);

  // ------ Computed Quantities ------
  // Temperatures
  T_in = Medium.temperature(state_in);
  T_out = Medium.temperature(state_out);
  Tm = (T_in + T_out)/2;

  // Densities
  rho_in = Medium.density(state_in);
  rho_out = Medium.density(state_out);
  rhom = (rho_in + rho_out)/2;

  // Volumiv flow rate
  Qv_in = Q_in / rho_in;
  Qv_out = Q_out / rho_out;
  Qvm = Qm / rhom;

  // ------ Conservation equations ------
  Q_in + Q_out = - DM; // Q_out < 0 if Q_out flows out of component, DM = Q out of component - Q into component
  P_out - P_in = DP;
  Q_in*h_in + Q_out*h_out = - W; // W = (Qh) out of component - (Qh) into component
  Q_in*Xi_in + Q_out*Xi_out = - DXi; // W = (QXi) out of component - (QXi) into component

end PartialTransportModel;
