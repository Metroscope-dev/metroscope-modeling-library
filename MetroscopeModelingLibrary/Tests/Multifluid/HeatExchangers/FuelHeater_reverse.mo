within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model FuelHeater_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  // Boundary conditions
  input Real P_hot_source(start=47, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=8) "kg/s";
  input Real T_hot_source(start=230) "J/kg";

  input Real P_cold_source(start=30, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=12) "kg/s";
  input Real T_cold_source(start = 30, min = 0, nominal = 50) "degC";
  //input Units.SpecificEnthalpy h_cold_source(start=1e6) "J/kg";
  // Parameters
  parameter String QCp_max_side = "undefined"; // On fuel heater, QCp_hot may be close to QCp_cold
  parameter Utilities.Units.Area S=100;

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_hot;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
  input Real P_cold_out(start = 30, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 47, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_hot_out(start = 80, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth
  //input Real T_cold_out(start = 200, nominal = 200)"degC";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={64,40})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-90})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.Sensors.Fuel.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_hot_out_sensor(h_0=5.75e5)
                                                                                  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-40})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,-68})));
  MultiFluid.HeatExchangers.FuelHeater fuelHeater annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
equation
  // Boundary conditions
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = T_hot_source+273.15;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  //cold_source.h_out = h_cold_source;
  cold_source.Q_out = - Q_cold_source;
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Parameters
  fuelHeater.S = S;

  // Inputs for calibration
  T_hot_out_sensor.T_degC = T_hot_out;
  //cold_sink.T_in = T_cold_out +273.15;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  fuelHeater.Kth = Kth;
  fuelHeater.Kfr_hot = Kfr_hot;
  fuelHeater.Kfr_cold = Kfr_cold;

  connect(cold_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{59,0},{40,0}}, color={213,213,0}));
  connect(T_hot_out_sensor.C_out, P_hot_out_sensor.C_in) annotation (Line(points={{-20,-50},{-20,-58}},
                                                                                                      color={28,108,200}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{-20,-85},{-20,-78}},
                                                                                              color={28,108,200}));
  connect(fuelHeater.C_cold_out, P_cold_out_sensor.C_in) annotation (Line(points={{6,0},{20,0}}, color={213,213,0}));
  connect(fuelHeater.C_hot_out, T_hot_out_sensor.C_in) annotation (Line(points={{-8,-8},{-8,-20},{-20,-20},{-20,-30}}, color={28,108,200}));
  connect(hot_source.C_out, fuelHeater.C_hot_in) annotation (Line(points={{59,40},{0,40},{0,8}}, color={28,108,200}));
  connect(fuelHeater.C_cold_in, cold_source.C_out) annotation (Line(points={{-14,0},{-59,0}}, color={213,213,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater_reverse;
