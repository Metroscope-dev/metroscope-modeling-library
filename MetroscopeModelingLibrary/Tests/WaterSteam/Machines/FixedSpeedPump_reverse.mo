within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model FixedSpeedPump_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=20e5);
  input Utilities.Units.Temperature source_T(start=150 + 273.15);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-1000);

  // Calibrated parameters
  output Real pump_hn;
  output Real pump_rh;

  // Calibration inputs
  input Utilities.Units.Pressure pump_P_out(start=60e5);
  input Utilities.Units.Temperature pump_T_out(start=150.5 + 273.15);

  MetroscopeModelingLibrary.WaterSteam.Machines.FixedSpeedPump
                                                      fixedSpeedPump
                                                           annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-30,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-70,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={80,0})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor pump_T_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor pump_P_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;

  // Calibrated parameters
  fixedSpeedPump.hn = pump_hn;
  fixedSpeedPump.rh = pump_rh;

  // Inputs for calibration
  pump_T_out_sensor.T = pump_T_out;
  pump_P_out_sensor.P = pump_P_out;

  connect(fixedSpeedPump.C_in, source.C_out) annotation (Line(points={{-40,0},{-65,0}}, color={28,108,200}));
  connect(fixedSpeedPump.C_out, pump_T_out_sensor.C_in) annotation (Line(points={{-20,0},{-10,0}}, color={28,108,200}));
  connect(pump_T_out_sensor.C_out, pump_P_out_sensor.C_in) annotation (Line(points={{10,0},{30,0}}, color={28,108,200}));
  connect(pump_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{50,0},{75,0}}, color={28,108,200}));
end FixedSpeedPump_reverse;
