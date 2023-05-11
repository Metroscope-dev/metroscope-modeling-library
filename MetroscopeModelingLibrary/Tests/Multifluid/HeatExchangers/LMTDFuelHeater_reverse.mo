within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model LMTDFuelHeater_reverse
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  // Boundary conditions
  input Real P_hot_source(start=47, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=8.6) "kg/s";
  input Real T_hot_source(start=230) "J/kg";

  input Real P_cold_source(start=30, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=12) "kg/s";
  input Real T_cold_source(start = 30, min = 0, nominal = 50) "degC";
  //input Units.SpecificEnthalpy h_cold_source(start=1e6) "J/kg";
  // Parameters
  parameter Utilities.Units.Area S=100;

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_hot;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
  input Real P_cold_out(start = 30, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 47, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 97, nominal = 100) "degC";// Outlet temperature on cold side, to calibrate Kth
  MultiFluid.HeatExchangers.LMTDFuelHeater
                                       lMTDFuelHeater                          annotation (Placement(transformation(extent={{-36,-34},{40,34}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={18,50})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-12,-92})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{76,-10},{96,10}})));
  MetroscopeModelingLibrary.Sensors.Fuel.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{36,-10},{56,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-12,-70})));
  MetroscopeModelingLibrary.Sensors.Fuel.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent={{58,-10},{78,10}})));
equation
    // Boundary conditions
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = T_hot_source+273.15;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Parameters
  lMTDFuelHeater.S = S;

    // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  lMTDFuelHeater.Kth = Kth;
  lMTDFuelHeater.Kfr_hot = Kfr_hot;
  lMTDFuelHeater.Kfr_cold = Kfr_cold;
  connect(lMTDFuelHeater.C_hot_in, hot_source.C_out) annotation (Line(points={{17.2,23.8},{18,23.8},{18,45}}, color={28,108,200}));
  connect(lMTDFuelHeater.C_cold_in, cold_source.C_out) annotation (Line(points={{-24.6,0},{-49,0}}, color={213,213,0}));
  connect(lMTDFuelHeater.C_cold_out, P_cold_out_sensor.C_in) annotation (Line(points={{28.6,0},{36,0}}, color={213,213,0}));
  connect(hot_sink.C_in,P_hot_out_sensor. C_out) annotation (Line(points={{-12,-87},{-12,-80}},
                                                                                              color={28,108,200}));
  connect(lMTDFuelHeater.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{-13.2,-23.8},{-12,-23.8},{-12,-60}}, color={28,108,200}));
  connect(cold_sink.C_in, T_cold_out_sensor.C_out) annotation (Line(points={{81,0},{78,0}}, color={213,213,0}));
  connect(T_cold_out_sensor.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{58,0},{56,0}}, color={213,213,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end LMTDFuelHeater_reverse;
