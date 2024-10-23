within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSource
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  replaceable package Medium =
      MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Initialization parameters
  parameter Units.Pressure P_0 = 1e5;
  parameter Units.MassFlowRate Q_0 = 1000;
  parameter Units.SpecificEnthalpy h_0=5e5;
  parameter Units.Temperature T_0 = 300;

  // Input Quantities
  Inputs.InputSpecificEnthalpy h_out(start=h_0);
  Inputs.InputMassFraction Xi_out[Medium.nXi];
  Inputs.InputPressure P_out(start=P_0);
  Units.NegativeMassFlowRate Q_out(start=-Q_0);

  Units.NegativeVolumeFlowRate Qv_out(start=-Q_0/1000);

  // Computed quantities
  Units.Temperature T_out(start=T_0);
  Medium.ThermodynamicState state_out;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidOutlet C_out(Q(start=-Q_0), P(start=P_0), h_outflow(start=h_0))
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
  annotation (Icon(graphics={
        Ellipse(
          extent={{-80,60},{40,-60}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5,
          pattern=LinePattern.None,
          lineColor={0,0,0})}));
end FluidSource;
