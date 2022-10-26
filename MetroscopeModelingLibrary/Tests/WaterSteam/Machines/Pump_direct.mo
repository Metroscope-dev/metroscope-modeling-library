within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model Pump_direct
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=20e5);
  input Units.Temperature source_T(start= 150 + 273.15);
  input Units.NegativeMassFlowRate source_Q(start=-1000);
  input Real pump_VRot(start=1400);

  // Component parameters
  parameter Real pump_VRotn = 1400;
  parameter Real pump_a3 = 444;
  parameter Real pump_b3 = 0.93;
  parameter Units.Yield pump_rhmin = 0.20;

  .MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
  MetroscopeModelingLibrary.Sensors.Outline.VRotSensor pump_VRot_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-48})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
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
  pump.rm = 0.85;
  pump.a1 = 0;
  pump.a2 = 0;
  pump.a3 = pump_a3;
  pump.b1 = 0;
  pump.b2 = 0;
  pump.b3 = pump_b3;
  pump.rhmin = pump_rhmin;

  connect(pump.C_in, source.C_out) annotation (Line(points={{-10,0},{-61,0}}, color={28,108,200}));
  connect(pump.C_out, sink.C_in) annotation (Line(points={{10,0},{71,0}}, color={28,108,200}));
  connect(pump.VRot, pump_VRot_sensor.VRot) annotation (Line(points={{0,-12},{0,-37.8},{1.77636e-15,-37.8}}, color={0,0,127}));
  connect(pump.C_power, powerSource.C_out) annotation (Line(points={{0,10.8},{0,28},{-8.88178e-16,28},{-8.88178e-16,45.2}}, color={244,125,35}));
end Pump_direct;
