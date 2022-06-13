within MetroscopeModelingLibrary.Tests.MoistAir.Pipes;
model Pipe_direct
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
      // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

    // Parameters
  parameter Units.FrictionCoefficient Kfr = 100;
  parameter Units.Height delta_z = 0;

  MetroscopeModelingLibrary.MoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  pipe.Kfr = Kfr;
  pipe.delta_z = delta_z;

  connect(pipe.C_in, source.C_out) annotation (Line(points={{-10,0},{-37,0}}, color={85,170,255}));
  connect(pipe.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_direct;
