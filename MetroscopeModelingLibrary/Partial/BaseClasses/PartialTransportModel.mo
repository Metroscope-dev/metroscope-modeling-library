within MetroscopeModelingLibrary.Partial.BaseClasses;
partial model PartialTransportModel "Basic fluid transport brick for all components"
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // ------ Initialization parameters ------
  // Enthalpies
  parameter Units.SpecificEnthalpy h_in_0 = 1e6;
  parameter Units.SpecificEnthalpy h_out_0 = 1e6;
  parameter Units.Temperature T_in_0 = 300;
  parameter Units.Temperature T_out_0 = 300;
  // Mass Flow Rate
  parameter Units.MassFlowRate Q_in_0 = 100;
  parameter Units.MassFlowRate Q_out_0 = - Q_in_0;
  // Pressure
  parameter Units.Pressure P_in_0 = 1e5;
  parameter Units.Pressure P_out_0 = 1e5;
  // Mass fractions
  parameter Units.MassFraction Xi_in_0[Medium.nXi] = zeros(Medium.nXi);
  parameter Units.MassFraction Xi_out_0[Medium.nXi] = zeros(Medium.nXi);

  // ------ Input Quantities ------
  // Enthalpies
  Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
  Units.SpecificEnthalpy h_out(start=h_out_0) "Outlet specific enthalpy";
  Units.SpecificEnthalpy hm(start=hm_0) "Average specific enthalpy";
  // Mass flow rate
  Units.MassFlowRate Q_in(start=Q_in_0) "Inlet Mass flow rate";
  Units.MassFlowRate Q_out(start=Q_out_0) "Outlet Mass flow rate";
  Units.MassFlowRate Qm(start=Qm_0) "Mean Mass flow rate";
  // Pressures
  Units.Pressure P_in(start=P_in_0) "Inlet Pressure";
  Units.Pressure P_out(start=P_out_0) "Outlet Pressure";
  Units.Pressure Pm(start=Pm_0) "Average fluid pressure";
  // Mass fractions
  Units.MassFraction Xi_in[Medium.nXi](start=Xi_in_0) "Inlet species mass fraction";
  Units.MassFraction Xi_out[Medium.nXi](start=Xi_out_0) "Outlet species mass fraction";
  Units.MassFraction Xim[Medium.nXi](start=Xim_0) "Outlet species mass fraction";
  //extends PartialTransportXi(redeclare package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium);

  // ------ Computed Quantities ------
  // Densities
  Units.Density rho_in(start=998) "Inlet Fluid density";
  Units.Density rho_out(start=998) "Outlet Fluid density";
  Units.Density rhom(start=998) "Average Fluid density";

  // Volumic flow rates
  Units.VolumeFlowRate Qv_in(start=0.1) "inlet volume flow rate";
  Units.VolumeFlowRate Qv_out(start=0.1) "outlet volume flow rate";
  Units.VolumeFlowRate Qvm(start=0.1) "Mean volume flow rate";

  // Temperatures
  Units.Temperature T_in(start=T_in_0) "Fluid temperature";
  Units.Temperature T_out(start=T_out_0) "Fluid temperature";
  Units.Temperature Tm(start=Tm_0) "Fluid temperature";

  // ------ States ------
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  // ------ Conservation variables ------
  Units.MassFlowRate DM(nominal=0, start=0); // Mass Loss
  Units.DifferentialPressure DP(nominal=0, start=0); // Pressure Loss
  Units.Power W(nominal=0, start=0); // Heat Loss
  Units.MassFraction DXi[Medium.nXi] "species mass fraction variation in component";

  // ------ Connectors ------
  replaceable Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-120,-20},{-80,20}})));
  replaceable Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{80,-20},{120,20}})));
protected
  parameter Units.SpecificEnthalpy hm_0 = (h_in_0 + h_out_0)/2;
  parameter Units.Temperature Tm_0 = (T_in_0 + T_out_0)/2;
  parameter Units.MassFlowRate Qm_0 = (Q_in_0 + Q_out_0)/2;
  parameter Units.Pressure Pm_0 = (P_in_0 + P_out_0)/2;
  parameter Units.MassFraction Xim_0[Medium.nXi] = (Xi_in_0 + Xi_out_0)/2;
  parameter Medium.ThermodynamicState statem_0 = Medium.setState_phX(Pm_0, hm_0, Xim_0);
  parameter Units.Density rhom_0 = Medium.density(statem_0);
  parameter Units.VolumeFlowRate Qvm_0 = Qm_0 / rhom_0;
equation
  // ------ Input Quantities ------
  // Enthalpies
  h_out = C_out.h_outflow;
  h_in = inStream(C_in.h_outflow);
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
  C_in.h_outflow = 1e5; // Never used arbitrary value
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
  Q_in + Q_out = DM;
  P_out - P_in = DP;
  Q_in*h_in + Q_out*h_out = W;
  Q_in*Xi_in + Q_out*Xi_out = DXi;
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,60},{100,-62}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1)}),
        primitivesVisible=primitivesVisible);
end PartialTransportModel;
