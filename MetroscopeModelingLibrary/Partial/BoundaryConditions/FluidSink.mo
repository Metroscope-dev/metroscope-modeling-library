within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  replaceable package Medium =
      MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Input Quantities
  Units.SpecificEnthalpy h_in;
  Units.MassFraction Xi_in[Medium.nXi];
  Inputs.InputPressure P_in;
  Units.PositiveMassFlowRate Q_in(start=1e3);
  Units.PositiveVolumeFlowRate Qv_in(start=1);

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
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-16,36},{57,-37}}, color={0,0,0},
          thickness=1),
        Line(points={{-16,-36},{57,37}}, color={0,0,0},
          thickness=1)}));
end FluidSink;
