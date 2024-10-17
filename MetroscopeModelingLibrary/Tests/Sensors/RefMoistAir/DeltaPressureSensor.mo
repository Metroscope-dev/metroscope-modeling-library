within MetroscopeModelingLibrary.Tests.Sensors.RefMoistAir;
model DeltaPressureSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Sensors.RefMoistAir.DeltaPressureSensor DP_sensor annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{52,-10},{72,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoHFlowModel MoistAirIsoHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.P_out = source_P;
  source.relative_humidity = 0.1;
  DP_sensor.DP = 0.5e5;

  assert(abs(DP_sensor.DP_bar - 0.5) < 1e-5, "DP_sensor should detect 1 barA");
  assert(abs(sink.P_in - source.P_out - DP_sensor.DP) < 1e-5, "Pressure difference from source to sink should be the one detected by DP_sensor");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(MoistAirIsoHFlowModel.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
  connect(MoistAirIsoHFlowModel.C_out, sink.C_in) annotation (Line(points={{10,0},{57,0}}, color={0,255,128}));
  connect(DP_sensor.C_out, sink.C_in) annotation (Line(points={{10,20},{40,20},{40,0},{57,0}}, color={0,255,128}));
  connect(DP_sensor.C_in, source.C_out) annotation (Line(points={{-10,20},{-28,20},{-28,0},{-39,0}}, color={0,255,128}));
end DeltaPressureSensor;
