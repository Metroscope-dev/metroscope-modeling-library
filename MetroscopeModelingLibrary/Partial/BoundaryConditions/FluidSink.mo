within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FluidSinkIcon;
  replaceable package Medium =
      MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Input Quantities
  Units.SpecificEnthalpy h_in;
  Inputs.InputMassFraction Xi_in[Medium.nXi];
  Units.Pressure P_in;
  Units.InletMassFlowRate Q_in(start=1e3);
  Units.InletVolumeFlowRate Qv_in(start=1);

  // Computed quantities
  Units.Temperature T_in;
  Medium.ThermodynamicState state_in;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidInlet C_in(Q(start=1e3))
    annotation (Placement(transformation(extent={{-62,-10},{-42,10}}),iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  // Connector
  C_in.P = P_in;
  C_in.Q = Q_in;
  inStream(C_in.h_outflow) = h_in;
  inStream(C_in.Xi_outflow) = Xi_in;

  // State
  state_in = Medium.setState_phX(P_in,h_in,Xi_in);

  // Computed quantities
  T_in = Medium.temperature(state_in);
  Qv_in = Q_in / Medium.density(state_in);

  // No flow reversal in stream connector
  C_in.h_outflow = 0; // Never used arbitrary value
  C_in.Xi_outflow = zeros(Medium.nXi); // No flow reversal
end FluidSink;
