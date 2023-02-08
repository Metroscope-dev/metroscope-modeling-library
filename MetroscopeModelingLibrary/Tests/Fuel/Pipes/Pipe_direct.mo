within MetroscopeModelingLibrary.Tests.Fuel.Pipes;
model Pipe_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;
    // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    // Parameters
  parameter Utilities.Units.FrictionCoefficient Kfr=0.1;
  parameter Utilities.Units.Height delta_z=0;

  MetroscopeModelingLibrary.Fuel.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{44,-10},{64,10}})));
equation
  // Boundary Conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

    // Parameters
  pipe.Kfr = Kfr;
  pipe.delta_z = delta_z;

  connect(pipe.C_in, source.C_out) annotation (Line(points={{-10,0},{-37,0}}, color={213,213,0}));
  connect(pipe.C_out, sink.C_in) annotation (Line(points={{10,0},{49,0}}, color={213,213,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_direct;
