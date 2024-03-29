within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model SteamTurbine_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=20e5);
  input Utilities.Units.SpecificEnthalpy source_h(start=2.7718e6);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-500);

  // Inputs for calibration
  input Real stodolaTurbine_P_out(start=5, unit="bar", nominal=15, min=0) "bar";
  input Real stodolaTurbine_W_out(start=100, unit="MW", nominal=100, min=0) "MW";

  // Calibrated parameters
  output Utilities.Units.Cst stodolaTurbine_Cst;
  output Utilities.Units.Yield stodolaTurbine_eta_is;

  .MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine stodolaTurbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor stodolaTurbine_P_out_sensor annotation (Placement(transformation(extent={{32,-10},{52,10}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor stodolaTurbine_W_out_sensor annotation (Placement(transformation(extent={{32,20},{52,40}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  // Calibrated parameters
  stodolaTurbine.Cst = stodolaTurbine_Cst;
  stodolaTurbine.eta_is = stodolaTurbine_eta_is;

  // Inputs for calibration
  stodolaTurbine_P_out_sensor.P_barA = stodolaTurbine_P_out;
  stodolaTurbine_W_out_sensor.W_MW = stodolaTurbine_W_out;
  connect(stodolaTurbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
  connect(stodolaTurbine_P_out_sensor.C_in, stodolaTurbine.C_out) annotation (Line(points={{32,0},{10,0}}, color={28,108,200}));
  connect(sink.C_in, stodolaTurbine_P_out_sensor.C_out) annotation (Line(points={{67,0},{52,0}}, color={28,108,200}));
  connect(stodolaTurbine.C_W_out, stodolaTurbine_W_out_sensor.C_in) annotation (Line(points={{10,8.4},{18,8.4},{18,30},{32,30}}, color={244,125,35}));
  connect(stodolaTurbine_W_out_sensor.C_out, power_sink.C_in) annotation (Line(points={{51.8,30},{67,30}}, color={244,125,35}));
end SteamTurbine_reverse;
