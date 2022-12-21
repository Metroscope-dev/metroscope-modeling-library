within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Superheater_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

  // Boundary conditions
  input Real P_hot_source(start=1.5, min=0, nominal=1) "barA";
  input Units.MassFlowRate Q_hot_source(start=586) "kg/s";
  input Real hot_source_h(start=600000) "J/kg";

  input Real P_cold_source(start=3.5, min=1.5, nominal=3.5) "barA";
  input Units.MassFlowRate Q_cold_source(start=11) "kg/s";
  input Real T_cold_source(start = 200, min = 130, nominal = 150) "degC";


  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Units.Area S = 10;
  parameter Units.Temperature nominal_cold_side_temperature_rise = 140;
  parameter Units.Temperature nominal_hot_side_temperature_drop = 3;

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;
  output Units.FrictionCoefficient Kfr_cold;


  // Calibration inputs
  input Real P_cold_out(start=3.5, min=1.5, nominal=3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start=1.5, min=0, nominal=1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 290, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  MultiFluid.HeatExchangers.Superheater superheater(QCp_max_side=QCp_max_side)
                                                                              annotation (Placement(transformation(extent={{-36,-34},{34,34}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={10,42})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-12,-70})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={-12,-52})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={-12,-38})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_hot_out_sensor annotation (Placement(transformation(extent={{40,-4},{48,4}})));
equation
    // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  superheater.S = S;
  superheater.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  superheater.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

    // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  superheater.Kth = Kth;
  superheater.Kfr_hot = Kfr_hot;
  superheater.Kfr_cold = Kfr_cold;


  connect(P_cold_out_sensor.C_out,T_cold_out_sensor. C_in)
    annotation (Line(points={{-12,-42},{-12,-48}},
                                                 color={28,108,200}));
  connect(cold_sink.C_in,T_cold_out_sensor. C_out)
    annotation (Line(points={{-12,-65},{-12,-56}},
                                                 color={28,108,200}));
  connect(superheater.C_hot_in, hot_source.C_out) annotation (Line(points={{-25.5,0},{-45,0}}, color={95,95,95}));
  connect(superheater.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{23.5,0},{40,0}}, color={95,95,95}));
  connect(hot_sink.C_in,P_hot_out_sensor. C_out)
    annotation (Line(points={{65,0},{48,0}}, color={95,95,95}));
  connect(P_cold_out_sensor.C_in, superheater.C_cold_out) annotation (Line(points={{-12,-34},{-12,-28.9},{-11.5,-28.9},{-11.5,23.8}},  color={28,108,200}));
  connect(cold_source.C_out, superheater.C_cold_in) annotation (Line(points={{10,37},{9.5,37},{9.5,23.8}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Superheater_reverse;
