within MetroscopeModelingLibrary.Tests.Sensors.RefMoistAir;
model FlowSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.PositiveMassFlowRate source_Q(start=100) "kg/s";

  MetroscopeModelingLibrary.Sensors.RefMoistAir.FlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = - source_Q;
  source.relative_humidity = 0.1;

  if not source_Q_sensor.faulty then // To avoid breaking FlowSensor_faulty test
    assert(abs(source_Q_sensor.Q - 100) < 1e-5, "Q_sensor should detect 100 kg/s");
  end if;
  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
  connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
end FlowSensor;
