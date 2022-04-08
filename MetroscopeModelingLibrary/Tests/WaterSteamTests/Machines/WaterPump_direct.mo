within MetroscopeModelingLibrary.Tests.WaterSteamTests.Machines;
model WaterPump_direct
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=2e5);
  input Units.Temperature source_T(start=20 + 273.15);
  input Units.OutletMassFlowRate source_Q(start=-100);
  input Real pump_VRot(start=1400);

  // Component parameters
  parameter Real pump_VRotn = 1400;
  parameter Real pump_rm = 0.85;
  parameter Real pump_a1 = -88.67;
  parameter Real pump_a2 = 0;
  parameter Real pump_a3 = 43.15;
  parameter Real pump_b1 = -3.7751;
  parameter Real pump_b2 = 3.61;
  parameter Real pump_b3 = -0.0075464;
  parameter Units.Yield pump_rhmin = 0.20;

  WaterSteam.Machines.WaterPump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  WaterSteam.BoundaryConditions.WaterSink sink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
  MetroscopeModelingLibrary.Sensors.Other.VRotSensor pump_VRot_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-48})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSource powerSource annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,50})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;
  pump_VRot_sensor.VRot = pump_VRot;

  // Component parameters
  pump.VRotn = pump_VRotn;
  pump.rm = pump_rm;
  pump.a1 = pump_a1;
  pump.a2 = pump_a2;
  pump.a3 = pump_a3;
  pump.b1 = pump_b1;
  pump.b2 = pump_b2;
  pump.b3 = pump_b3;
  pump.rhmin = pump_rhmin;

  connect(pump.C_in, source.C_out) annotation (Line(points={{-10,0},{-61,0}}, color={28,108,200}));
  connect(pump.C_out, sink.C_in) annotation (Line(points={{10,0},{71,0}}, color={28,108,200}));
  connect(pump.VRot, pump_VRot_sensor.VRot) annotation (Line(points={{0,-12},{0,-37.8},{1.77636e-15,-37.8}}, color={0,0,127}));
  connect(pump.C_power, powerSource.C_W_out) annotation (Line(points={{0,10.8},{0,28},{-8.88178e-16,28},{-8.88178e-16,45.2}}, color={244,125,35}));
end WaterPump_direct;
