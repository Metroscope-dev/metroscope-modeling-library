within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_without_flashTank_reverse
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
  input Real P_cold_out(start = 19, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 50, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 55, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  MultiFluid.HeatExchangers.Evaporator_without_flashTank
                                       evaporator_without_flashTank
                                                  annotation (Placement(transformation(extent={{-48,-50},{56,52}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent={{-60,40},{-48,52}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{-76,40},{-64,52}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlueGasesPressureSensor P_hot_out_sensor  annotation (Placement(transformation(extent={{60,-4},{68,4}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{62,46},{42,66}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-78,36},{-98,56}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-82,-10},{-62,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{78,-10},{98,10}})));
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
  evaporator_without_flashTank.S_vaporising = S;

  // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  evaporator_without_flashTank.Kth = Kth;
  evaporator_without_flashTank.Kfr_hot = Kfr_hot;
  evaporator_without_flashTank.Kfr_cold = Kfr_cold;

  connect(T_cold_out_sensor.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{-60,46},{-64,46}}, color={28,108,200}));
  connect(evaporator_without_flashTank.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{40.4,-0.02},{50.2,-0.02},{50.2,0},{60,0}}, color={95,95,95}));
  connect(evaporator_without_flashTank.C_cold_in, cold_source.C_out) annotation (Line(points={{19.6,36.7},{19.6,56},{47,56}}, color={28,108,200}));
  connect(P_cold_out_sensor.C_in, cold_steam_sink.C_in) annotation (Line(points={{-76,46},{-83,46}}, color={28,108,200}));
  connect(evaporator_without_flashTank.C_hot_in, hot_source.C_out) annotation (Line(points={{-32.4,-0.02},{-49.7,-0.02},{-49.7,0},{-67,0}}, color={95,95,95}));
  connect(P_hot_out_sensor.C_out,hot_sink. C_in) annotation (Line(points={{68,0},{83,0}}, color={95,95,95}));
  connect(evaporator_without_flashTank.C_cold_out, T_cold_out_sensor.C_out) annotation (Line(points={{-11.6,36.7},{-12,36.7},{-12,46},{-48,46}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_without_flashTank_reverse;
