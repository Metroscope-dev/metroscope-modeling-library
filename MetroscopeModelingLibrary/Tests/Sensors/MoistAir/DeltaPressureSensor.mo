within MetroscopeModelingLibrary.Tests.Sensors.MoistAir;
model DeltaPressureSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MoistAirTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Sensors.MoistAir.DeltaPressureSensor DP_sensor annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoHFlowModel MoistAirIsoHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
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
  connect(DP_sensor.C_out, sink.C_in) annotation (Line(points={{10,20},{20,20},{20,0},{43,0}}, color={28,108,200}));
  connect(source.C_out, MoistAirIsoHFlowModel.C_in) annotation (Line(points={{-43,0},{-10,0}}, color={28,108,200}));
  connect(MoistAirIsoHFlowModel.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
  connect(DP_sensor.C_in, MoistAirIsoHFlowModel.C_in) annotation (Line(points={{-10,20},{-20,20},{-20,0},{-10,0}}, color={28,108,200}));
end DeltaPressureSensor;
