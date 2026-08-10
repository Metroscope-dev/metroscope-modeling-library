within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_direct
   extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  input Real P_hot_source(start = 1.1, min = 1, nominal = 1) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 475) "degC";

  input Real P_cold_source(start = 130, min = 1.5, nominal = 3.5) "barA";
  input Real T_cold_source(start = 326, min = 130, nominal = 150) "degC";

  // Parameters
  parameter Utilities.Units.Area S = 50000 "m²";

  // Calibrated parameters
  parameter Utilities.Units.HeatExchangeCoefficient Kth = 34.43 "W/m².K";

  // Observables
  output Utilities.Units.MassFlowRate Q_cold_source "kg/s";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source
                                       cold_source annotation (Placement(transformation(extent={{96,30},{76,50}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink
                                     cold_steam_sink annotation (Placement(transformation(extent={{-68,70},{-88,90}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-102,-50},{-82,-30}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
  MultiFluid.HeatExchangers.Evaporator evaporator(feedwater_tank=false)
                                                                       annotation (Placement(transformation(extent={{-50,-90},{50,100}})));
equation
  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_source + 273.15;


  // Inputs for calibration
  cold_source.Q_out = - Q_cold_source;

  // Calibrated parameters
  evaporator.Kth = Kth;

  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-40,-40},{-87,-40}}, color={95,95,95}));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{40,-40},{77,-40}}, color={95,95,95}));
  connect(cold_source.C_out, evaporator.C_cold_in) annotation (Line(points={{81,40},{35,40}}, color={28,108,200}));
  connect(evaporator.C_cold_out, cold_steam_sink.C_in) annotation (Line(points={{-35,80},{-73,80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio = false)), Diagram(coordinateSystem(preserveAspectRatio = false)));
end Evaporator_direct;
