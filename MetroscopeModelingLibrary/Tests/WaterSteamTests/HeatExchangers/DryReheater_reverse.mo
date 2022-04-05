within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model DryReheater_reverse

  extends Modelica.Icons.Example;

  // Boundary conditions
  input Units.Pressure P_hot_source(start=11e5, min=0, nominal=11e5) "Pa";
  input Units.Pressure P_cold_source(start=50e5, min=0, nominal=50e5) "Pa";
  input Units.OutletMassFlowRate Q_cold_source(start=-500) "kg/s";
  input Units.SpecificEnthalpy cold_source_h_out(start=5.5e5) "J/kg";
  input Units.SpecificEnthalpy hot_source_h_out(start=2.5e6) "J/kg";

  // Component Parameters
  parameter Units.Area S = 100;

  // Observables for calibration
  input Real P_hot_sink(start=10.9, min=0, nominal=11) "bar";
  input Real P_cold_sink(start=49.9, min=0, nominal=50) "bar";
  input Real T_cold_sink(start=180, min=0, nominal=200) "degC";

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;
  output Units.FrictionCoefficient Kfr_cold;

  WaterSteam.BoundaryConditions.WaterSource cold_source
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.WaterSink cold_sink
    annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  WaterSteam.HeatExchangers.DryReheater dryReheater
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.WaterSource hot_source annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  WaterSteam.BoundaryConditions.WaterSink hot_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-60})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor T_cold_sink_sensor annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor P_hot_sink_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-32})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor P_cold_sink_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
equation
  // Boundary conditions
  hot_source.P_out = P_hot_source;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source;
  cold_source.h_out = cold_source_h_out;
  cold_source.Q_out = Q_cold_source;

  // Component parameters
  dryReheater.S_condensing = S;

  // Observables for calibration
  P_hot_sink_sensor.P_barA = P_hot_sink;
  P_cold_sink_sensor.P_barA = P_cold_sink;
  T_cold_sink_sensor.T_degC = T_cold_sink;

  // Calibrated parameters
  dryReheater.Kth = Kth;
  dryReheater.Kfr_hot = Kfr_hot;
  dryReheater.Kfr_cold = Kfr_cold;

  connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, dryReheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(dryReheater.C_cold_out, P_cold_sink_sensor.C_in) annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_out, T_cold_sink_sensor.C_in) annotation (Line(points={{42,0},{48,0}}, color={28,108,200}));
  connect(T_cold_sink_sensor.C_out, cold_sink.C_in) annotation (Line(points={{68,0},{73,0}}, color={28,108,200}));
  connect(hot_sink.C_in, P_hot_sink_sensor.C_out) annotation (Line(points={{8.88178e-16,-55},{8.88178e-16,-48.5},{-1.77636e-15,-48.5},{-1.77636e-15,-42}}, color={28,108,200}));
  connect(dryReheater.C_hot_out, P_hot_sink_sensor.C_in) annotation (Line(points={{0,-8},{0,-15},{1.77636e-15,-15},{1.77636e-15,-22}}, color={28,108,200}));
end DryReheater_reverse;
