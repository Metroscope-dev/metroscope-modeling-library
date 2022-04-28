within MetroscopeModelingLibrary.Tests.Fuel.HeatExchangers;
model FuelHeater_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;
  // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real hot_source_h(start=0.7e6) "J/kg";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  //input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";
  input Units.SpecificEnthalpy h_cold_source(start=1e6) "J/kg";
  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Units.Area S = 100;

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;
  output Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
  input Real P_cold_out(start = 19, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 50, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_hot_out(start = 55, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth


  MultiFluid.HeatExchangers.FuelHeater fuelHeater annotation (Placement(transformation(extent={{-38,-34},{38,34}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={12,50})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,90})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.Sensors.Fuel.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{46,-10},{66,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_hot_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,42})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,68})));
equation
  // Boundary conditions
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  //cold_source.T_out = 273.15 + T_cold_source;
  cold_source.h_out = h_cold_source;
  cold_source.Q_out = - Q_cold_source;
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Parameters
  fuelHeater.S = S;

    // Inputs for calibration
  T_hot_out_sensor.T_degC = T_hot_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  fuelHeater.Kth = Kth;
  fuelHeater.Kfr_hot = Kfr_hot;
  fuelHeater.Kfr_cold = Kfr_cold;

  connect(fuelHeater.C_hot_in, hot_source.C_out) annotation (Line(points={{11.4,23.8},{12,23.8},{12,45}}, color={28,108,200}));
  connect(fuelHeater.C_cold_in, cold_source.C_out) annotation (Line(points={{-26.6,0},{-51,0}}, color={213,213,0}));
  connect(fuelHeater.C_cold_out, P_cold_out_sensor.C_in) annotation (Line(points={{26.6,0},{46,0}}, color={213,213,0}));
  connect(cold_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{79,0},{66,0}}, color={213,213,0}));
  connect(fuelHeater.C_hot_out, T_hot_out_sensor.C_in) annotation (Line(points={{-11.4,23.8},{-11.4,27.9},{-12,27.9},{-12,32}}, color={28,108,200}));
  connect(T_hot_out_sensor.C_out, P_hot_out_sensor.C_in) annotation (Line(points={{-12,52},{-12,58}}, color={28,108,200}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{-12,85},{-12,78}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater_reverse;
