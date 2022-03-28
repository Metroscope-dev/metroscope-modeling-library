within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model FluidSink
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Input Quantities
  Inputs.InputSpecificEnthalpy h_in(start=1e5);
  Inputs.InputMassFraction Xi_in[Medium.nXi];
  Units.Pressure P_in(start=1e5);
  Units.MassFlowRate Q_in(start=500);

  Units.VolumeFlowRate Qv_in(start=0.5);

  // Computed quantities
  Units.Temperature T_in(start=300);
  Medium.ThermodynamicState state_in;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidInlet C_in
    annotation (Placement(transformation(extent={{-62,-10},{-42,10}}),iconTransformation(extent={{-62,-10},{-42,10}})));
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
  C_in.h_outflow = 1e5; // Never used arbitrary value
  C_in.Xi_outflow = zeros(Medium.nXi); // No flow reversal
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-90,0},{-62,0},{-76,10}}),
        Line(points={{-76,-10},{-62,0}}),
        Ellipse(
          extent={{-41,59},{79,-61}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-31,49},{69,-51}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-16,34},{54,-36}}, color={0,0,0}),
        Line(points={{-16,-36},{54,34}}, color={0,0,0})}),
                                            Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end FluidSink;
