within MetroscopeModelingLibrary.Tests.MoistAirTests.BaseClassesTests;
model MoistAirIsoHFlowModelTest
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
  MoistAir.BaseClasses.MoistAirIsoHFlowModel moist_air_IsoHFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
  MoistAir.BoundaryConditions.MoistAirSource moist_air_Source annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
  MoistAir.BoundaryConditions.MoistAirSink moist_air_Sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirPressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirFlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
equation
  moist_air_IsoHFlowModel.DP_input = 0;

  moist_air_Source.relative_humidity = 0.1;
  moist_air_Source.h_out = 1e3;

  moist_air_PressureSensor.P = 1e5;
  moist_air_FlowSensor.Q = 100;

  assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In IsoHFlowModel, DM should be 0");
  assert(abs(moist_air_Sink.Q_in*moist_air_Sink.h_in + moist_air_Source.Q_out*moist_air_Source.h_out) <= 1e-5, "In IsoHFlowModel, W should be 0");
  connect(moist_air_IsoHFlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{51,0},{57.2,0},{57.2,0},{76,0}},             color={85,170,255}));
  connect(moist_air_PressureSensor.C_out,moist_air_FlowSensor. C_in) annotation (Line(points={{-50,0},{-48,0},{-48,0.1},{-44,0.1},{-44,0},{-38,0}},
                                                                                                                      color={85,170,255}));
  connect(moist_air_PressureSensor.C_in, moist_air_Source.C_out) annotation (Line(points={{-70,0},{-73.82,0},{-73.82,0},{-80.5,0}},        color={85,170,255}));
  connect(moist_air_IsoHFlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{5,0},{-11.5,0},{-11.5,0},{-18,0}},               color={85,170,255}));
end MoistAirIsoHFlowModelTest;
