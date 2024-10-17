within MetroscopeModelingLibrary.Tests.Sensors.RefMoistAir;
model PressureSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
equation
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.relative_humidity = 0.1;
  P_sensor.P = 1e5;

  assert(abs(P_sensor.P_barA - 1) < 1e-5, "P_sensor should detect 1 barA");
  assert(abs(P_sensor.P_psiA - 14.50377377) < 1e-3, "P_sensor should detect 14.5 psiA");
  assert(abs(P_sensor.P_kPaA - 100) < 1e-3, "P_sensor should detect 100 kPaA");
  assert(abs(P_sensor.P_MPaA - 0.1) < 1e-3, "P_sensor should detect 0.1 MPaA");

  assert(abs(P_sensor.P_kPaG) < 1e-3, "P_sensor should detect 0 kPaG");
  assert(abs(P_sensor.P_MPaG) < 1e-3, "P_sensor should detect 0 MPaG");
  assert(abs(P_sensor.P_barG) < 1e-5, "P_sensor should detect 0 barG");
  assert(abs(P_sensor.P_psiG) < 1e-3, "P_sensor should detect 0 psiG");

  assert(abs(P_sensor.P_inHg - 29.530058647) < 1e-5, "P_sensor should detect 29.53 inHg");
  assert(abs(P_sensor.P_mbar - 1000) < 1e-5, "P_sensor should detect 1000 mbar");

  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(P_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
  connect(P_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{59,0}}, color={0,255,128}));
end PressureSensor;
