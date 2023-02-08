within MetroscopeModelingLibrary.Tests.MoistAir.Pipes;
model Leak
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MoistAirTestIcon;
      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=10e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";
  input Utilities.Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";


  MetroscopeModelingLibrary.MoistAir.Pipes.Leak leak annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;
  sink.P_in = sink_P;

  connect(leak.C_in, source.C_out) annotation (Line(points={{-10,0},{-37,0}}, color={85,170,255}));
  connect(leak.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Leak;
