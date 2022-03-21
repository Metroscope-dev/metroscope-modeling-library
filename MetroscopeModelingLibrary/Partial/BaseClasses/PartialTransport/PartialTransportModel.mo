within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportModel "Basic fluid transport brick for all components"
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  // ------ Input Quantities ------
  // Enthalpies
  extends PartialTransportH(Medium = Medium);
  // Mass flow rate
  extends PartialTransportQ(Medium = Medium);
  // Pressures
  extends PartialTransportP(Medium = Medium);
  // Mass fractions
  extends PartialTransportXi(Medium = Medium);


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
  Units.Temperature T_in(start=290) "Fluid temperature";
  Units.Temperature T_out(start=291) "Fluid temperature";
  Units.Temperature Tm(start=290) "Fluid temperature";

  // ------ States ------
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  // ------ Conservation variables ------
  Units.MassFlowRate DM(nominal=0, start=0); // Mass Loss
  Units.DifferentialPressure DP(nominal=0, start=0); // Pressure Loss
  Units.Power W(nominal=0, start=0); // Heat Loss
  Units.MassFraction DXi[Medium.nXi] "species mass fraction variation in component";

protected
  parameter Medium.ThermodynamicState statem_0 = Medium.setState_phX(Pm_0, hm_0, Xim_0);
  parameter Units.Density rhom_0 = Medium.density(statem_0);
  parameter Units.VolumeFlowRate Qvm_0 = Qm_0 / rhom_0;
equation
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
end PartialTransportModel;
