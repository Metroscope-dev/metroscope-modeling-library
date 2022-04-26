within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;
  input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
  input Units.MassFlowRate Q_hot_source(start=586);
  input Real hot_source_h(start=494000);

  input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
  input Units.MassFlowRate Q_cold_source(start=96);
  input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

  // Parameters
  parameter Units.Area S = 10;


  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;
  output Units.FrictionCoefficient Kfr_cold;

  // Calibration inputs
  input Real P_cold_out(start = 3.5, min=1.5, nominal=3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start=1, min=1, nominal=1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real Q_cold_liq_out(start = 97, min = 80, nominal = 97) "kg/s"; // Outlet temperature on cold side, to calibrate Kth

  MetroscopeModelingLibrary.Sensors.FlueGases.FlueGasesPressureSensor P_hot_out_sensor  annotation (Placement(transformation(extent={{56,-4},{64,4}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{58,46},{38,66}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-82,36},{-102,56}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-58,-72},{-78,-52}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Q_cold_liquid_out annotation (Placement(transformation(extent={{-38,-72},{-58,-52}})));
  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-40,-38},{38,42}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{-56,56},{-76,36}})));
equation

  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source;
  cold_source.T_out =  T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  evaporator.S_vaporising = S;

  // Inputs for calibration
  //T_cold_out_sensor.T_degC = T_cold_out;
  Q_cold_liquid_out.Q = Q_cold_liq_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  evaporator.Kth = Kth;
  evaporator.Kfr_hot = Kfr_hot;
  evaporator.Kfr_cold = Kfr_cold;

  connect(P_hot_out_sensor.C_out, hot_sink.C_in) annotation (Line(points={{64,0},{79,0}}, color={95,95,95}));
  connect(cold_liquid_sink.C_in, Q_cold_liquid_out.C_out) annotation (Line(points={{-63,-62},{-58,-62}}, color={28,108,200}));
  connect(evaporator.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{26.3,1.2},{26.3,0},{56,0}}, color={95,95,95}));
  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-28.3,1.2},{-28.3,0},{-71,0}}, color={95,95,95}));
  connect(evaporator.C_cold_liq_out, Q_cold_liquid_out.C_in) annotation (Line(points={{-28.3,-34},{-28.3,-62},{-38,-62}}, color={28,108,200}));
  connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{18.5,30},{18.5,56},{43,56}}, color={28,108,200}));
  connect(evaporator.C_cold_vap_out, P_cold_out_sensor.C_in) annotation (Line(points={{-28.3,38},{-28.3,46},{-56,46}}, color={28,108,200}));
  connect(cold_steam_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{-87,46},{-76,46}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_reverse;
