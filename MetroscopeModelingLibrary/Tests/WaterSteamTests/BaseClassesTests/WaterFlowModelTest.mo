within MetroscopeModelingLibrary.Tests.WaterSteamTests.BaseClassesTests;
model WaterFlowModelTest
  WaterSteam.BaseClasses.WaterFlowModel waterFlowModel annotation (Placement(transformation(extent={{11,-23},{57,23}})));
  WaterSteam.BoundaryConditions.WaterSource waterSource annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{61,-20},{103,20}})));
  Sensors.WaterSteam.WaterPressureSensor waterPressureSensor annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Sensors.WaterSteam.WaterFlowSensor waterFlowSensor annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
equation
  waterFlowModel.W_input = 0;
  waterFlowModel.DP_input = 0;

  waterSource.h_out = 1e6;

  waterPressureSensor.P = 1e5;
  waterFlowSensor.Q = 100;

  //waterSink.C_in.h_outflow = 0;

  assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In flow model, DM should be 0");
  connect(waterFlowModel.C_out, waterSink.C_in) annotation (Line(points={{57,-0.23},{57,0},{71.5,0}}, color={28,108,200}));
  connect(waterSource.C_out, waterPressureSensor.C_in) annotation (Line(points={{-71.64,0},{-71.64,-0.1},{-60,-0.1}}, color={28,108,200}));
  connect(waterPressureSensor.C_out, waterFlowSensor.C_in) annotation (Line(points={{-40,-0.1},{-24,-0.1}}, color={28,108,200}));
  connect(waterFlowModel.C_in, waterFlowSensor.C_out) annotation (Line(points={{11,-0.23},{8,-0.23},{8,-0.1},{-4,-0.1}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}}),
     Rectangle(
          extent={{22,-26},{84,-95}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{66,-44},{100,-78}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{4,-43},{40,-79}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                     Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterFlowModelTest;
