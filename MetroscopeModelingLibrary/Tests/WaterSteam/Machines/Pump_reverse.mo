within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model Pump_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=20e5);
  input Utilities.Units.Temperature source_T(start=150 + 273.15);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-1000);

  // Component parameters
  parameter Real pump_VRot = 1400;
  parameter Real pump_VRotn = 1400;
  parameter Utilities.Units.Yield pump_rhmin=0.20;

  // Calibrated parameters
  output Real pump_a3;
  output Real pump_b3;

  // Calibration inputs
  input Utilities.Units.Pressure pump_P_out(start=60e5);
  input Utilities.Units.Temperature pump_T_out(start=150.5 + 273.15);

  .MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-30,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-70,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={80,0})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor pump_T_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor pump_P_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
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
  pump.rm = 0.85;
  pump.a1 = 0;
  pump.a2 = 0;
  pump.b1 = 0;
  pump.b2 = 0;
  pump.rhmin = 0.2;

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
