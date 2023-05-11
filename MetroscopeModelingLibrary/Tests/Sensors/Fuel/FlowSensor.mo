within MetroscopeModelingLibrary.Tests.Sensors.Fuel;
model FlowSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Utilities.Units.PositiveMassFlowRate source_Q(start=100) "kg/s";

  MetroscopeModelingLibrary.Sensors.Fuel.FlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = - source_Q;

  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  if not source_Q_sensor.faulty then // To avoid breaking FlowSensor_faulty test
    assert(abs(source_Q_sensor.Q - 100) < 1e-5, "Q_sensor should detect 100 kg/s");
  end if;
  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
end FlowSensor;
