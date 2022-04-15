within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Pipe_reverse
  extends Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

    // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  parameter Units.Height delta_z = 1;

  // Inputs for calibration
  input Real P_out(start=9) "barA";

  // Parameters for calibration
  output Units.FrictionCoefficient Kfr;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlueGasesPressureSensor P_out_sensor annotation (Placement(transformation(extent={{28,-10},{48,10}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  // Parameters
  pipe.delta_z = delta_z;

  // Inputs for calibration
  P_out_sensor.P_barA = P_out;

  // Parameters for calibration
  pipe.Kfr = Kfr;

  connect(source.C_out, pipe.C_in) annotation (Line(points={{-23,0},{-10,0}}, color={95,95,95}));
  connect(pipe.C_out, P_out_sensor.C_in) annotation (Line(points={{10,0},{28,0}}, color={95,95,95}));
  connect(P_out_sensor.C_out, sink.C_in) annotation (Line(points={{48,0},{73,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_reverse;