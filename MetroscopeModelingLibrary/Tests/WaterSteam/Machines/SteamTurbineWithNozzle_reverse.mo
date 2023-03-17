within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model SteamTurbineWithNozzle_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=0.2e5);
  input Utilities.Units.SpecificEnthalpy source_h(start=2.328e6);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-800);

  // Inputs for calibration
  input Real turbine_W_out(start=108.1, unit="MW", nominal=100, min=0) "MW";

  // Calibrated parameters
  output Utilities.Units.Yield turbine_eta_nz;

  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbineWithNozzle
                                                              turbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor turbine_W_out_sensor annotation (Placement(transformation(extent={{32,20},{52,40}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  // Turbine parameters
  turbine.area_nz = 25;
  turbine.Cst = 2;
  turbine.eta_is = 0.93;

  // Calibrated parameters
  turbine.eta_nz = turbine_eta_nz;

  // Inputs for calibration
  turbine_W_out_sensor.W_MW = turbine_W_out;

  connect(turbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
  connect(turbine.C_W_out, turbine_W_out_sensor.C_in) annotation (Line(points={{6,8},{18,8},{18,30},{32,30}},      color={244,125,35}));
  connect(turbine_W_out_sensor.C_out, power_sink.C_in) annotation (Line(points={{51.8,30},{67,30}}, color={244,125,35}));
  connect(turbine.C_out, sink.C_in) annotation (Line(points={{10,0},{67,0}}, color={28,108,200}));
end SteamTurbineWithNozzle_reverse;
