within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model Filter_direct

    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.Temperature source_T(start=11) "degC";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

    // Parameters
  parameter Utilities.Units.FrictionCoefficient Kfr=0.01;
  parameter Utilities.Units.Height delta_z=0;

  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Pipes.Filter filter
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  // Boundary Conditions
  source.T_out = source_T + 273.15;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  filter.Kfr = Kfr;
  filter.delta_z = delta_z;

  connect(source.C_out, filter.C_in)
    annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
  connect(sink.C_in, filter.C_out)
    annotation (Line(points={{39,0},{10,0}}, color={0,255,128}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Filter_direct;
