within MetroscopeModelingLibrary.Tests.WaterSteamTests.Machines;
model StodolaTurbine_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=20e5);
  //input Units.Temperature source_T(start=20 + 273.15);
  input Units.SpecificEnthalpy source_h(start=2.7718e6);
  input Units.OutletMassFlowRate source_Q(start=-100);

  // Calibration observables
  input Real stodolaTurbine_P_out(start=15, unit="bar", nominal=15, min=0) "bar";
  input Real stodolaTurbine_W_out(start=2.6, unit="MW", nominal=100, min=0) "MW";

  // Calibrated parameters
  output Units.Cst stodolaTurbine_Cst;
  output Units.Yield stodolaTurbine_eta_is;

  WaterSteam.Machines.StodolaTurbine stodolaTurbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  WaterSteam.BoundaryConditions.WaterSink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor stodolaTurbine_P_out_sensor annotation (Placement(transformation(extent={{32,-10},{52,10}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor stodolaTurbine_W_out_sensor annotation (Placement(transformation(extent={{32,20},{52,40}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  // Component parameters
  stodolaTurbine.area_nz = 1;
  stodolaTurbine.eta_nz = 1;

  // Calibrated parameters
  stodolaTurbine.Cst = stodolaTurbine_Cst;
  stodolaTurbine.eta_is = stodolaTurbine_eta_is;

  // Calibration observables
  stodolaTurbine_P_out_sensor.P_barA = stodolaTurbine_P_out;
  stodolaTurbine_W_out_sensor.W_MW = stodolaTurbine_W_out;
  connect(stodolaTurbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
  connect(stodolaTurbine_P_out_sensor.C_in, stodolaTurbine.C_out) annotation (Line(points={{32,0},{10,0}}, color={28,108,200}));
  connect(sink.C_in, stodolaTurbine_P_out_sensor.C_out) annotation (Line(points={{67,0},{52,0}}, color={28,108,200}));
  connect(stodolaTurbine.C_W_out, stodolaTurbine_W_out_sensor.C_in) annotation (Line(points={{10,8.4},{18,8.4},{18,30},{32,30}}, color={244,125,35}));
  connect(stodolaTurbine_W_out_sensor.C_out, power_sink.C_W_in) annotation (Line(points={{51.8,30},{67,30}}, color={244,125,35}));
end StodolaTurbine_reverse;
