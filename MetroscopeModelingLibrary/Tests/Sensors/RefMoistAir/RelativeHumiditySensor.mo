within MetroscopeModelingLibrary.Tests.Sensors.RefMoistAir;
model RelativeHumiditySensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Real relative_humidity(start=10) "%";

  MetroscopeModelingLibrary.Sensors.RefMoistAir.RelativeHumiditySensor
                                                            H_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
equation
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.P_out = 1e5;
  H_sensor.relative_humidity_pc = relative_humidity;

  connect(H_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
  connect(H_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
end RelativeHumiditySensor;
