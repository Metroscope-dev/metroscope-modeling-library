within MetroscopeModelingLibrary.Tests.WaterSteamTests.BaseClassesTests;
model WaterIsoPHFlowModelTest
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;
  WaterSteam.BaseClasses.WaterIsoPHFlowModel waterIsoPHFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
  WaterSteam.BoundaryConditions.Source waterSource annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
  WaterSteam.BoundaryConditions.Sink waterSink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor waterPressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor waterFlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
equation
  waterSource.h_out = 1e6;

  waterPressureSensor.P = 1e5;
  waterFlowSensor.Q = 100;

  assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In IsoPHFlowModel, DM should be 0");
  assert(abs(waterSink.Qv_in + waterSource.Qv_out) <= 1e-5, "In IsoPHFlowModel, DV should be 0");
  assert(abs(waterSource.P_out - waterSink.P_in) <= 1e-5, "In IsoPHFlowModel, DP should be 0");
  assert(abs(waterSink.Q_in*waterSink.h_in + waterSource.Q_out*waterSource.h_out) <= 1e-5, "In IsoPHFlowModel, W should be 0");
  connect(waterIsoPHFlowModel.C_out, waterSink.C_in) annotation (Line(points={{51,0},{57.2,0},{57.2,0},{76,0}},             color={28,108,200}));
  connect(waterPressureSensor.C_out,waterFlowSensor. C_in) annotation (Line(points={{-50,0},{-48,0},{-48,0.1},{-44,0.1},{-44,0},{-38,0}},
                                                                                                            color={28,108,200}));
  connect(waterPressureSensor.C_in, waterSource.C_out) annotation (Line(points={{-70,0},{-73.82,0},{-73.82,0},{-80.5,0}},        color={28,108,200}));
  connect(waterIsoPHFlowModel.C_in, waterFlowSensor.C_out) annotation (Line(points={{5,0},{-11.5,0},{-11.5,0},{-18,0}},               color={28,108,200}));
end WaterIsoPHFlowModelTest;
