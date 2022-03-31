within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model WaterPipeTest_reverse
  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.SpecificEnthalpy h_source(start=1e6);
  input Real source_P(start=10, min=0, nominal=10) "barA";
  input Units.MassFlowRate source_Q(start=100) "kg/s";
  input Units.Height z1(start=0) "m";

  // Input: Observables
  input Units.DifferentialPressure DP_f(start=-0.5e5) "Pa";
  input Units.DifferentialPressure DP_z(start=0.6e5) "Pa";

  // Output: Component parameters
  output Units.FrictionCoefficient Kfr;
  output Units.Height z2;

  // Components
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.WaterSink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  WaterSteam.Pipes.WaterPipe pipe annotation (Placement(transformation(extent={{-6.5,-16.3333},{26.5,16.3333}})));

  Sensors.WaterSteam.WaterPressureSensor source_P_sensor annotation (Placement(transformation(extent={{-66,-6},{-54,6}})));
  Sensors.WaterSteam.WaterFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
equation
  // Boundary conditions
  source.h_out = h_source;
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;
  pipe.z1 = z1;

  // Input: Observables
  pipe.DP_f = DP_f;
  pipe.DP_z = DP_z;

  // Output: Component parameters
  pipe.Kfr = Kfr;
  pipe.z2 = z2;
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-66,0},{-84.2,0},{-84.2,7.5e-06},{-85,7.5e-06}},   color={28,108,200}));
  connect(pipe.C_in, source_Q_sensor.C_out) annotation (Line(points={{-6.5,0},{-30,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_in, source_P_sensor.C_out) annotation (Line(points={{-42,0},{-54,0}}, color={28,108,200}));
  connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{26.5,0}}, color={28,108,200}));
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
end WaterPipeTest_reverse;
