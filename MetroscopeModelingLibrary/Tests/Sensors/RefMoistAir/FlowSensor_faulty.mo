within MetroscopeModelingLibrary.Tests.Sensors.RefMoistAir;
model FlowSensor_faulty
  extends FlowSensor(source_Q_sensor(faulty = true));

  Utilities.Units.MassFlowRate Fault_mass_flow_rate_bias(start=0);
equation
  Fault_mass_flow_rate_bias = 0 + 50 * time; // bias of -50 means that real flow = measured flow - 50
  source_Q_sensor.mass_flow_rate_bias = Fault_mass_flow_rate_bias;

  connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={28,108,200}));
end FlowSensor_faulty;
