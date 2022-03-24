within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model Source
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Input Quantities
  Inputs.InputSpecificEnthalpy h_out(start=1e5);
  Inputs.InputMassFraction Xi_out[Medium.nXi];
  Inputs.InputPressure P_out(start=1e5);
  Units.MassFlowRate Q_out(start=500);

  Units.VolumeFlowRate Qv_out(start=0.5);

  // Computed quantities
  Units.Temperature T_out(start=300);
  Medium.ThermodynamicState state_out;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidOutlet C_out
    annotation (Placement(transformation(extent={{34,-10},{54,10}}),  iconTransformation(extent={{34,-10},{54,10}})));
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
          extent={{-74,58},{46,-62}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None),
        Line(points={{42,0},{80,0},{66,10}}),
        Line(points={{66,-10},{80,0}})}));
end Source;
