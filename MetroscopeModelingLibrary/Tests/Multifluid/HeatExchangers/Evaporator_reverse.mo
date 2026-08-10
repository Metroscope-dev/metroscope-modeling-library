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

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;

  // Calibration inputs
  input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source
                                       cold_source annotation (Placement(transformation(extent = {{76,50},{56,70}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink
                                     cold_steam_sink annotation (Placement(transformation(extent={{-76,110},{-96,130}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_cold_source_sensor annotation (Placement(transformation(extent={{60,50},{40,70}})));
  MultiFluid.HeatExchangers.Evaporator evaporator(feedwater_tank=false)
                                                  annotation (Placement(transformation(extent={{-64,-50},{36,140}})));
equation
  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_source + 273.15;


  // Inputs for calibration
  Q_cold_source_sensor.Q = Q_cold_source;

  // Calibrated parameters
  evaporator.Kth = Kth;


  connect(Q_cold_source_sensor.C_in, cold_source.C_out) annotation (Line(points={{60,60},{61,60}},   color = {28,108,200},
      thickness = 1));
  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-54,0},{-75,0}}, color={95,95,95}));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{26,0},{73,0}}, color={95,95,95}));
  connect(Q_cold_source_sensor.C_out, evaporator.C_cold_in) annotation (Line(points={{40,60},{21,60},{21,80}}, color={28,108,200}));
  connect(evaporator.C_cold_out, cold_steam_sink.C_in) annotation (Line(points={{-49,120},{-81,120}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio = false, extent={{-100,-100},{100,160}})),
                                                                   Diagram(coordinateSystem(preserveAspectRatio = false, extent={{-100,-100},{100,160}})));
end Evaporator_reverse;
