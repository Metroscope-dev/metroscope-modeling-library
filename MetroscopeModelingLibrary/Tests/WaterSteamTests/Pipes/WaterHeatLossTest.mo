within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model WaterHeatLossTest
  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Real source_P(start=2, min=0, nominal=2) "barA";
  input Units.MassFlowRate source_Q(start=1) "kg/s";

  // Input: Observables
  input Real W_input(start=1e5, min=0, nominal=100);

  // Components
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.WaterSink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  WaterSteam.Pipes.WaterHeatLoss heat_loss annotation (Placement(transformation(extent={{-8.5,-16.3333},{24.5,16.3333}})));

  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor source_P_sensor annotation (Placement(transformation(extent={{-66,-6},{-54,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
equation
  // Boundary conditions
  source.h_out = source_h;
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;

  // Observables
  heat_loss.W_input = W_input;
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-66,-3.75e-06},{-84.2,-3.75e-06},{-84.2,3.75e-06},{-85,3.75e-06}},
                                                                                                                           color={28,108,200}));
  connect(heat_loss.C_in, source_Q_sensor.C_out) annotation (Line(points={{-8.5,0},{-30,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_in, source_P_sensor.C_out) annotation (Line(points={{-42,0},{-54,0}}, color={28,108,200}));
  connect(sink.C_in, heat_loss.C_out) annotation (Line(points={{85,0},{24.5,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),
                                            Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterHeatLossTest;
