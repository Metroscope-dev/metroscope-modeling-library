within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSource
  extends MetroscopeModelingLibrary.Utilities.Icons.BoundaryConditions.FluidSourceIcon;
  replaceable package Medium =
      MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Input Quantities
  Inputs.InputSpecificEnthalpy h_out;
  Inputs.InputMassFraction Xi_out[Medium.nXi];
  Inputs.InputPressure P_out;
  Units.NegativeMassFlowRate Q_out(start=-1e3);

  Units.NegativeVolumeFlowRate Qv_out(start=-1);

  // Computed quantities
  Units.Temperature T_out;
  Medium.ThermodynamicState state_out;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidOutlet C_out(Q(start=-1e3))
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
