within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model DryReheater_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Utilities.Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

  // Component Parameters
  parameter Utilities.Units.Area S=100;
  parameter Utilities.Units.FrictionCoefficient Kfr_hot=0;

  // Observables for calibration
  input Real P_cold_sink(start=49, min=0, nominal=50) "bar";
  input Real T_cold_sink(start=70, min=0, nominal=200) "degC";

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dryReheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-36})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_sink_sensor annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_sink_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
equation

  // Boundary conditions
  hot_source.P_out = P_hot_source*1e5;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  // Component parameters
  dryReheater.S = S;
  dryReheater.Kfr_hot = Kfr_hot;

  // Observables for calibration
  P_cold_sink_sensor.P_barA = P_cold_sink;
  T_cold_sink_sensor.T_degC = T_cold_sink;

  // Calibrated parameters
  dryReheater.Kth = Kth;
  dryReheater.Kfr_cold = Kfr_cold;

  connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, dryReheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(dryReheater.C_cold_out, P_cold_sink_sensor.C_in) annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_out, T_cold_sink_sensor.C_in) annotation (Line(points={{42,0},{48,0}}, color={28,108,200}));
  connect(T_cold_sink_sensor.C_out, cold_sink.C_in) annotation (Line(points={{68,0},{73,0}}, color={28,108,200}));
  connect(dryReheater.C_hot_out, hot_sink.C_in) annotation (Line(points={{0,-8},
          {0,-19.5},{8.88178e-16,-19.5},{8.88178e-16,-31}}, color={28,108,200}));
end DryReheater_reverse;
