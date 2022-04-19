within MetroscopeModelingLibrary.Tests.WaterSteamTests.Machines;
model Pump_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=2e5);
  input Units.Temperature source_T(start=20 + 273.15);
  input Units.NegativeMassFlowRate source_Q(start=-100);

  // Component parameters
  parameter Real pump_VRot = 1400;
  parameter Real pump_VRotn = 1400;
  parameter Real pump_rm = 0.85;
  parameter Real pump_a1 = -88.67;
  parameter Real pump_a2 = 0;
  parameter Real pump_b1 = -3.7751;
  parameter Real pump_b2 = 3.61;
  parameter Units.Yield pump_rhmin = 0.20;

  // Calibrated parameters
  output Real pump_a3;
  output Real pump_b3;

  // Calibration inputs
  input Units.Pressure pump_P_out(start=6e5);
  input Units.Temperature pump_T_out(start=20 + 273.15);

  WaterSteam.Machines.Pump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-30,0})));
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-70,0})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={80,0})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor pump_T_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor pump_P_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
  MetroscopeModelingLibrary.Sensors.Outline.VRotSensor pump_VRot_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-46})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-30,42})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;
  pump_VRot_sensor.VRot = pump_VRot; // Could be replaced by power fed to component

  // Component parameters
  pump.VRotn = pump_VRotn;
  pump.rm = pump_rm;
  pump.a1 = pump_a1;
  pump.a2 = pump_a2;
  pump.b1 = pump_b1;
  pump.b2 = pump_b2;
  pump.rhmin = pump_rhmin;

  // Calibrated parameters
  pump.a3 = pump_a3;
  pump.b3 = pump_b3;

  // Inputs for calibration
  pump_T_out_sensor.T = pump_T_out;
  pump_P_out_sensor.P = pump_P_out;

  connect(pump.C_in, source.C_out) annotation (Line(points={{-40,0},{-65,0}}, color={28,108,200}));
  connect(pump.C_out, pump_T_out_sensor.C_in) annotation (Line(points={{-20,0},{-10,0}}, color={28,108,200}));
  connect(pump_T_out_sensor.C_out, pump_P_out_sensor.C_in) annotation (Line(points={{10,0},{30,0}}, color={28,108,200}));
  connect(pump_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{50,0},{75,0}}, color={28,108,200}));
  connect(pump.VRot, pump_VRot_sensor.VRot) annotation (Line(points={{-30,-12},{-30,-35.8}}, color={0,0,127}));
  connect(pump.C_power, powerSource.C_out) annotation (Line(points={{-30,10.8},{-30,37.2}}, color={244,125,35}));
end Pump_reverse;
