within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_reverse
   extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  input Real P_hot_source(start = 1.1, min = 1, nominal = 1) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 475) "degC";

  input Real P_cold_source(start = 130, min = 1.5, nominal = 3.5) "barA";
  input Real T_cold_source(start = 326, min = 130, nominal = 150) "degC";

  // Parameters
  parameter Utilities.Units.Area S = 50000;
  parameter Real x_steam_out = 1; // Set to 1 when the whole quantity of water is evaporated

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;

  // Calibration inputs
  input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source
                                       cold_source annotation (Placement(transformation(extent = {{76,50},{56,70}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink
                                     cold_steam_sink annotation (Placement(transformation(extent = {{-62,50},{-82,70}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent = {{-82,-10},{-62,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent = {{78,-10},{98,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_cold_source_sensor annotation (Placement(transformation(extent = {{40,50},{20,70}})));
  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent = {{-10,-10},{10,10}})));
equation
  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_source + 273.15;

  // Parameters
  evaporator.S = S;
  evaporator.x_steam_out = x_steam_out;
  evaporator.Kfr_hot = 0;
  evaporator.Kfr_cold = 0;

  // Inputs for calibration
  Q_cold_source_sensor.Q = Q_cold_source;

  // Calibrated parameters
  evaporator.Kth = Kth;


  connect(Q_cold_source_sensor.C_in, cold_source.C_out) annotation (Line(points = {{40,60},{61,60}}, color = {28,108,200},
      thickness = 1));
  connect(Q_cold_source_sensor.C_out, evaporator.C_cold_in) annotation (Line(points = {{20,60},{4,60},{4,8}}, color = {28,108,200},
      thickness = 1));
  connect(cold_steam_sink.C_in, evaporator.C_cold_out) annotation (Line(points = {{-67,60},{-4,60},{-4,8}}, color = {28,108,200},
      thickness = 1));
  connect(hot_source.C_out, evaporator.C_hot_in) annotation (Line(points = {{-67,0},{-10,0}}, color = {95,95,95},
      thickness = 1));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points = {{10,0},{83,0}}, color = {95,95,95},
      thickness = 1));
  annotation (Icon(coordinateSystem(preserveAspectRatio = false)), Diagram(coordinateSystem(preserveAspectRatio = false)));
end Evaporator_reverse;
