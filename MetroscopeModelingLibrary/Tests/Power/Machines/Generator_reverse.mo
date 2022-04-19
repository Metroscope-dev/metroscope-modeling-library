within MetroscopeModelingLibrary.Tests.Power.Machines;
model Generator_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary Condition
  input Units.NegativePower source_W(start=-1e6);

  // Observable for calibration
  input Real sink_W(start=0.99, nominal=100, min=0);

  // Component parameter
  output Units.Yield generator_eta;

  MetroscopeModelingLibrary.Power.Machines.Generator generator annotation (Placement(transformation(extent={{-46,-26},{40,26}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor sink_W_sensor annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  // Boundary conditions
  source.W_out = source_W;

  // Observable for calibration
  sink_W_sensor.W_MW = sink_W;

  // Component parameter
  generator.eta = generator_eta;
  connect(generator.C_in, source.C_out) annotation (Line(points={{-29.66,0},{-63.2,0}}, color={244,125,35}));
  connect(generator.C_out, sink_W_sensor.C_in) annotation (Line(points={{27.1,0},{38,0}}, color={244,125,35}));
  connect(sink_W_sensor.C_out, sink.C_in) annotation (Line(points={{57.8,0},{65,0}}, color={244,125,35}));
end Generator_reverse;
