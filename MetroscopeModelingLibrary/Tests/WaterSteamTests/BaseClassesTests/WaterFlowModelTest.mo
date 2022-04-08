within MetroscopeModelingLibrary.Tests.WaterSteamTests.BaseClassesTests;
model WaterFlowModelTest
  WaterSteam.BaseClasses.WaterFlowModel waterFlowModel annotation (Placement(transformation(extent={{7,-23},{53,23}})));
  WaterSteam.BoundaryConditions.WaterSource waterSource annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor waterPressureSensor annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor waterFlowSensor annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
equation
  waterFlowModel.W_input = 0;
  waterFlowModel.DP_input = 0;

  waterSource.h_out = 1e6;

  waterPressureSensor.P = 1e5;
  waterFlowSensor.Q = 100;

  assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In flow model, DM should be 0");
  connect(waterFlowModel.C_out, waterSink.C_in) annotation (Line(points={{53,0},{69.5,0}},            color={28,108,200}));
  connect(waterSource.C_out, waterPressureSensor.C_in) annotation (Line(points={{-70.5,0},{-60,0}},                   color={28,108,200}));
  connect(waterPressureSensor.C_out, waterFlowSensor.C_in) annotation (Line(points={{-40,0},{-34,0},{-34,0.025},{-30,0.025},{-30,0},{-20,0}},
                                                                                                            color={28,108,200}));
  connect(waterFlowModel.C_in, waterFlowSensor.C_out) annotation (Line(points={{7,0},{0,0}},                             color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterFlowModelTest;
