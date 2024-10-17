within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Superheater_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  // Boundary conditions
  input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 600) "degC";

  input Real P_cold_source(start = 130, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";
  input Real T_cold_source(start = 410, min = 130, nominal = 150) "degC";

  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Utilities.Units.Area S = 10000;
  parameter Utilities.Units.Temperature nominal_cold_side_temperature_rise = 105;
  parameter Utilities.Units.Temperature nominal_hot_side_temperature_drop = 35;
  parameter Utilities.Units.FrictionCoefficient Kfr_hot = 0;

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

  // Calibration inputs
  input Real P_cold_out(start = 129.5, min = 1.5, nominal = 3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real T_cold_out(start = 515, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation = 0,
        origin={66,40})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation = 0,
        origin = {-66,40})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent = {{-76,-10},{-56,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{56,-10},{76,10}})));
  MultiFluid.HeatExchangers.Superheater superheater annotation (Placement(transformation(extent = {{0,-10},{20,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent = {{0,30},{-20,50}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent = {{-30,30},{-50,50}})));
equation
    // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_source + 273.15;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  superheater.S = S;
  superheater.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  superheater.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

  // Observables
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;

  // Calibrated parameters
  superheater.Kth = Kth;
  superheater.Kfr_hot = Kfr_hot;
  superheater.Kfr_cold = Kfr_cold;

  connect(superheater.C_hot_in, hot_source.C_out) annotation (Line(points = {{0,0},{-61,0}}, color={95,95,95},
      thickness=1));
  connect(T_cold_out_sensor.C_in, superheater.C_cold_out) annotation (Line(points = {{0,40},{6,40},{6,8}}, color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(T_cold_out_sensor.C_out, P_cold_out_sensor.C_in) annotation (Line(points = {{-20,40},{-30,40}}, color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater.C_cold_in, cold_source.C_out) annotation (Line(points={{14,8},{14,40},{61,40}},   color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater.C_hot_out, hot_sink.C_in) annotation (Line(points={{20,0},{61,0}},   color={95,95,95},
      thickness=1));
  connect(P_cold_out_sensor.C_out, cold_sink.C_in) annotation (Line(points = {{-50,40},{-61,40}}, color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  annotation (Icon(coordinateSystem(preserveAspectRatio = false)), Diagram(coordinateSystem(preserveAspectRatio = false)));
end Superheater_reverse;
