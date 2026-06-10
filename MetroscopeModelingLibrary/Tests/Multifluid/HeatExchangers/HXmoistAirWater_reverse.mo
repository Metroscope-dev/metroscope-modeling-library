within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model HXmoistAirWater_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

 // Boundary conditions
  input Real P_hot_source(start=3, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=31.4) "kg/s";
  input Real T_hot_source(start=9.03, nominal = 40, unit="degC");

  input Real P_cold_source(start=0.985, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=700) "kg/s";
  input Real T_cold_source(start = 6.44, nominal = 10,unit="degC");
  //input Real h_cold_source(start = 9e3, min = 0, nominal = 10e4) "degC";
  //input Real T_cold_sink(  start = 7.54, nominal = 10, unit= "degC");
  //input Real moistAir_relative_humidity( start=0.55, nominal=1) "%";

  // Parameters
  parameter String QCp_max_side = "cold";



  // Calibration inputs
  input Real P_cold_out(start = 0.9, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 2.8, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_hot_out(start = 7.8, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  MultiFluid.HeatExchangers.HXmoistAirWater hXmoistAirWater(QCp_max_side = QCp_max_side,
    Q_cold_0=700,
    Q_hot_0=0.15,
    P_cold_in_0=100000,
    cold_side(
    h_in(           start =  9e3)))                                                                 annotation (Placement(transformation(extent={{-26,18},{22,-22}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-30,-96},{-10,-76}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{12,64},{32,84}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(Q_out(start=-700))
                                                                           annotation (Placement(transformation(extent={{-90,-12},{-70,8}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{72,-12},{92,8}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor(
    Q_0=0.15,
    P_0=300000,
    h_0=1e5)                                                                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,34})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_hot_out_sensor(
    Q_0=0.15,
    P_0=300000,
    h_0=1e5,
    T_0=296.15)                                                                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,60})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor P_cold_out_sensor(Q_0=700, h_0=9e3)
                                                                              annotation (Placement(transformation(extent={{48,-12},{68,8}})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(extent={{-56,14},{-48,22}}), iconTransformation(extent={{-178,36},{-158,56}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(extent={{-50,-22},{-42,-14}}), iconTransformation(extent={{-166,-2},{-146,18}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={38,26}), iconTransformation(extent={{-178,36},{-158,56}})));
equation
  // Boundary conditions
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = T_hot_source +273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;


  cold_source.Q_out = - Q_cold_source;
  cold_source.relative_humidity = 55/100; //moistAir_relative_humidity;


  // Inputs for calibration
  T_hot_out_sensor.T_degC = T_hot_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;



  connect(hot_source.C_out, hXmoistAirWater.C_hot_in) annotation (Line(points={{-15,-86},{-2,-86},{-2,-18}}, color={28,108,200}));
  connect(hXmoistAirWater.C_cold_in, cold_source.C_out) annotation (Line(points={{-26,-2},{-75,-2}},   color={85,170,255}));
  connect(P_hot_out_sensor.C_in, hXmoistAirWater.C_hot_out) annotation (Line(points={{0,24},{-2,8},{-2,14}},          color={28,108,200}));
  connect(hot_sink.C_in, T_hot_out_sensor.C_out) annotation (Line(points={{17,74},{5.55112e-16,74},{5.55112e-16,70}},
                                                                                                           color={28,108,200}));
  connect(T_hot_out_sensor.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{-5.55112e-16,50},{-5.55112e-16,47},{5.55112e-16,47},{5.55112e-16,44}}, color={28,108,200}));
  connect(hXmoistAirWater.C_cold_out, P_cold_out_sensor.C_in) annotation (Line(points={{22,-2},{48,-2}},   color={85,170,255}));
  connect(P_cold_out_sensor.C_out, cold_sink.C_in) annotation (Line(points={{68,-2},{77,-2}}, color={85,170,255}));
  connect(hXmoistAirWater.Kth, Kth) annotation (Line(points={{-28.4,10},{-52,10},{-52,18}}, color={0,0,127}));
  connect(hXmoistAirWater.Kfr_cold, Kfr_cold) annotation (Line(points={{-28.4,-14},{-38,-14},{-38,-18},{-46,-18}}, color={0,0,127}));
  connect(hXmoistAirWater.Kfr_hot, Kfr_hot) annotation (Line(points={{24.4,10},{38,10},{38,26}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end HXmoistAirWater_reverse;
