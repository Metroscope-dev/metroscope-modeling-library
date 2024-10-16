within MetroscopeModelingLibrary.Tests.Sensors.MoistAir;
model RelativeHumiditySensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Real relative_humidity(start=10) "%";

  MetroscopeModelingLibrary.Sensors.MoistAir.RelativeHumiditySensor
                                                            H_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.P_out = 1e5;
  H_sensor.relative_humidity_pc = relative_humidity;

  connect(H_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
  connect(H_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
end RelativeHumiditySensor;
