within MetroscopeModelingLibrary.Tests.Power.Machines;
model Generator_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.PowerTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary Condition
  input Units.NegativePower source_W(start=-1e6);

  // Component parameter
  parameter Units.Yield generator_eta = 0.99;

  MetroscopeModelingLibrary.Power.Machines.Generator generator annotation (Placement(transformation(extent={{-46,-26},{40,26}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  // Boundary conditions
  source.W_out = source_W;

  // Component parameter
  generator.eta = generator_eta;
  connect(generator.C_in, source.C_out) annotation (Line(points={{-29.66,0},{-63.2,0}}, color={244,125,35}));
  connect(generator.C_out, sink.C_in) annotation (Line(points={{27.1,0},{65,0}}, color={244,125,35}));
end Generator_direct;
