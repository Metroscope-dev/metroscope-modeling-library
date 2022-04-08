within MetroscopeModelingLibrary.Tests.MoistAirTests.BaseClassesTests;
model MoistAirFlowModelTest
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
  MoistAir.BaseClasses.MoistAirFlowModel moist_air_FlowModel annotation (Placement(transformation(extent={{7,-23},{53,23}})));
  MoistAir.BoundaryConditions.MoistAirSource moist_air_Source annotation (Placement(transformation(extent={{-108.5,-18.5},{-71.5,18.5}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirPressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirFlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  MoistAir.BoundaryConditions.MoistAirSink moist_air_Sink annotation (Placement(transformation(extent={{61.5,-20},{102.5,20}})));
equation
  moist_air_FlowModel.W_input = 0;
  moist_air_FlowModel.DP_input = 0;

  moist_air_Source.relative_humidity = 0.1;
  moist_air_Source.h_out = 1e3;

  moist_air_PressureSensor.P_barA = 0.9;
  moist_air_FlowSensor.Q = 100;

  assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In flow model, DM should be 0");
  connect(moist_air_Source.C_out, moist_air_PressureSensor.C_in) annotation (Line(points={{-80.75,0},{-72,0},{-72,0}},          color={85,170,255}));
  connect(moist_air_PressureSensor.C_out, moist_air_FlowSensor.C_in) annotation (Line(points={{-52,0},{-50,0},{-50,0.1},{-46,0.1},{-46,0},{-40,0}},
                                                                                                                      color={85,170,255}));
  connect(moist_air_FlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{7,0},{4.5,0},{4.5,0},{-20,0}},           color={85,170,255}));
  connect(moist_air_FlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{53,0},{62.375,0},{62.375,0},{71.75,0}},         color={85,170,255}));
end MoistAirFlowModelTest;
