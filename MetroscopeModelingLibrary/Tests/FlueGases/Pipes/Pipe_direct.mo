within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Pipe_direct
  extends Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

    // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  parameter Units.FrictionCoefficient Kfr = 1e2;
  parameter Units.Height delta_z = 1;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  // Parameters
  pipe.Kfr = Kfr;
  pipe.delta_z = delta_z;



  connect(source.C_out, pipe.C_in) annotation (Line(points={{-23,0},{-10,0}}, color={95,95,95}));
  connect(pipe.C_out, sink.C_in) annotation (Line(points={{10,0},{23,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_direct;
