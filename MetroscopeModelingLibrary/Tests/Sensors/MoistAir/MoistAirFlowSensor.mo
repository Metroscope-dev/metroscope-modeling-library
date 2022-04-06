within MetroscopeModelingLibrary.Tests.Sensors.MoistAir;
model MoistAirFlowSensor
  extends Modelica.Icons.Example;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
  input Units.InletMassFlowRate source_Q(start=100) "kg/s";

  MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.MoistAirSource source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.MoistAirSink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source_Q_sensor.Q = source_Q;
  source.relative_humidity = 0.1;

  assert(abs(source_Q_sensor.Q - 100) < 1e-5, "T_sensor should detect 25 deg C");
  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
end MoistAirFlowSensor;
