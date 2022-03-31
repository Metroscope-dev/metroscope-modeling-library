within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSource
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FluidSourceIcon;
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Initialization parameters
  parameter Units.SpecificEnthalpy h_out_0 = 1e6;
  parameter Units.MassFraction Xi_out_0[Medium.nXi] = zeros(Medium.nXi);
  parameter Units.Pressure P_out_0 = 1e5;
  parameter Units.OutletMassFlowRate Q_out_0 = -500;
  parameter Units.VolumeFlowRate Qv_out_0 = Q_out_0 / 1000;

  // Computed quantities
  parameter Units.Temperature T_out_0 = 300;


  // Input Quantities
  Inputs.InputSpecificEnthalpy h_out(start=h_out_0);
  Inputs.InputMassFraction Xi_out[Medium.nXi](start=Xi_out_0);
  Inputs.InputPressure P_out(start=P_out_0);
  Units.MassFlowRate Q_out(max=0, start=Q_out_0, nominal=Q_out_0);

  Units.VolumeFlowRate Qv_out(max=0, start=Qv_out_0, nominal=Qv_out_0);

  // Computed quantities
  Units.Temperature T_out(start=T_out_0);
  Medium.ThermodynamicState state_out;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidOutlet C_out(Q(start=Q_out_0), P(start=P_out_0))
    annotation (Placement(transformation(extent={{40,-10},{60,10}}),  iconTransformation(extent={{40,-10},{60,10}})));
equation
  // Connector
  C_out.P = P_out;
  C_out.Q = Q_out;
  C_out.h_outflow = h_out;
  C_out.Xi_outflow = Xi_out;

  // State
  state_out = Medium.setState_phX(P_out, h_out, Xi_out);

  // Computed quantities
  T_out = Medium.temperature(state_out);
  Qv_out = Q_out / Medium.density(state_out);
end FluidSource;
