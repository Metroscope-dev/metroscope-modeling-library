within MetroscopeModelingLibrary.Tests.WaterSteamTests.BaseClassesTests;
model WaterIsoHFlowModelTest
  WaterSteam.BaseClasses.WaterIsoHFlowModel waterIsoPFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
  WaterSteam.BoundaryConditions.WaterSource waterSource annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
  WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
  Sensors.WaterSteam.WaterPressureSensor waterPressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Sensors.WaterSteam.WaterFlowSensor waterFlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
equation
  waterIsoPFlowModel.DP_input = 0;

  waterSource.h_out = 1e6;

  waterPressureSensor.P = 1e5;
  waterFlowSensor.Q = 100;

  assert(abs(waterSink.Q_in + waterSource.Q_out) <= 1e-5, "In IsoHFlowModel, DM should be 0");
  assert(abs(waterSink.Q_in*waterSink.h_in + waterSource.Q_out*waterSource.h_out) <= 1e-5, "In IsoHFlowModel, W should be 0");
  connect(waterIsoPFlowModel.C_out, waterSink.C_in) annotation (Line(points={{51,-0.23},{57.2,-0.23},{57.2,0},{75.6,0}},   color={28,108,200}));
  connect(waterPressureSensor.C_out,waterFlowSensor. C_in) annotation (Line(points={{-50,-0.1},{-38,-0.1}}, color={28,108,200}));
  connect(waterPressureSensor.C_in, waterSource.C_out) annotation (Line(points={{-70,-0.1},{-73.82,-0.1},{-73.82,0},{-81.64,0}}, color={28,108,200}));
  connect(waterIsoPFlowModel.C_in, waterFlowSensor.C_out) annotation (Line(points={{5,-0.23},{-11.5,-0.23},{-11.5,-0.1},{-18,-0.1}}, color={28,108,200}));
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
          extent={{22,-28},{84,-97}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{66,-46},{100,-80}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{4,-45},{40,-81}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),
       Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterIsoHFlowModelTest;
