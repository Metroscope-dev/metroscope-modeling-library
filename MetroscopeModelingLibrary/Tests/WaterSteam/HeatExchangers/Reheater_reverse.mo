within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Utilities.Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

  // Component Parameters
  parameter Utilities.Units.Area S_tot=100;
  parameter Utilities.Units.Fraction level=0.3;

  parameter Utilities.Units.FrictionCoefficient Kfr_hot=0;

  // Observables for calibration
  input Real P_cold_sink(start=49, min=0, nominal=50) "bar";
  input Real T_cold_sink(start=70, min=0, nominal=200) "degC";
  input Real T_drains(start=80, min=0, nominal=200) "degC";

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth_cond;
  output Utilities.Units.FrictionCoefficient Kfr_cold;
  output Utilities.Units.HeatExchangeCoefficient Kth_subc;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-56})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_sink_sensor annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_sink_sensor annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_drains_sensor(P_0=1100000)
                                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
equation

  // Boundary conditions
  hot_source.P_out = P_hot_source*1e5;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  // Component parameters
  reheater.S = S_tot;
  reheater.Kfr_hot = Kfr_hot;
  reheater.level = level;

  // Observables for calibration
  P_cold_sink_sensor.P_barA = P_cold_sink;
  T_cold_sink_sensor.T_degC = T_cold_sink;
  T_drains_sensor.T_degC = T_drains;

  // Calibrated parameters
  reheater.Kth_cond = Kth_cond;
  reheater.Kfr_cold = Kfr_cold;
  reheater.Kth_subc = Kth_subc;

  connect(hot_source.C_out, reheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, reheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(T_drains_sensor.C_in, reheater.C_hot_out) annotation (Line(points={{1.77636e-15,
          -20},{1.77636e-15,-14},{0,-14},{0,-8}}, color={28,108,200}));
  connect(T_drains_sensor.C_out, hot_sink.C_in) annotation (Line(points={{-1.77636e-15,
          -40},{-1.77636e-15,-45.5},{8.88178e-16,-45.5},{8.88178e-16,-51}},
        color={28,108,200}));
  connect(reheater.C_cold_out, T_cold_sink_sensor.C_in) annotation (Line(points={{16,0},{24,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_in, T_cold_sink_sensor.C_out) annotation (Line(points={{48,0},{44,0}}, color={28,108,200}));
  connect(cold_sink.C_in, P_cold_sink_sensor.C_out) annotation (Line(points={{73,0},{68,0}}, color={28,108,200}));
end Reheater_reverse;
