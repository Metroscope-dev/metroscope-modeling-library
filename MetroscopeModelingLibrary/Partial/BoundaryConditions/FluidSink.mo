within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSink
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Input Quantities
  Units.SpecificEnthalpy h_in;
  Inputs.InputMassFraction Xi_in[Medium.nXi];
  Units.Pressure P_in;
  Units.MassFlowRate Q_in;
  Units.VolumeFlowRate Qv_in;

  // Computed quantities
  Units.Temperature T_in;
  Medium.ThermodynamicState state_in;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidInlet C_in
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-90,0},{-62,0},{-76,10}}),
        Line(points={{-76,-10},{-62,0}}),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-15,35},{55,-35}}, color={0,0,0}),
        Line(points={{-15,-35},{55,35}}, color={0,0,0})}),
                                            Diagram(coordinateSystem(
          preserveAspectRatio=true)));
end FluidSink;
