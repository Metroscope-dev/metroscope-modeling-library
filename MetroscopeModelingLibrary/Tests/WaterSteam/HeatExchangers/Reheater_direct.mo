within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_direct

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // This component represents an NPP reheater where the drains from the upstream heater join the extraction steam of the top of the heater,
  // then pass through both the condensing and subcooling zones. It's identical to Reheater_reverse_connectors example except for the drains source.

  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Utilities.Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

  input Real Q_drains( start=95) "kg/s";
  input Real h_drains( start = 8e5) "J/kg";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater( input_specs = true)
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source Extr_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,54})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-56})));
  Utilities.Interfaces.RealOutput Kth_cond annotation (Placement(transformation(
          extent={{-34,-22},{-26,-14}}), iconTransformation(extent={{-254,-48},{
            -234,-28}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-40,12},{-32,20}}), iconTransformation(extent={{-250,-32},{-230,
            -12}})));
  Utilities.Interfaces.RealOutput Kth_subc annotation (Placement(transformation(
          extent={{-22,26},{-14,34}}), iconTransformation(extent={{-252,-40},{-232,
            -20}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_drains_sensor(
    sensor_function="Calibration",
    causality="Kth_subc",
    T_start=80,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_sink_sensor(
    sensor_function="Calibration",
    causality="Kth_cond",
    T_start=70,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_sink_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=49,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source Drains_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,44})));
  Utilities.Interfaces.RealOutput T_drains annotation (Placement(transformation(
          extent={{18,-34},{26,-26}}), iconTransformation(extent={{-254,-48},{-234,
            -28}})));
  Utilities.Interfaces.RealOutput T_cold_sink annotation (Placement(
        transformation(extent={{26,16},{34,24}}), iconTransformation(extent={{-254,
            -48},{-234,-28}})));
  Utilities.Interfaces.RealOutput P_cold_sink annotation (Placement(
        transformation(extent={{54,18},{62,26}}), iconTransformation(extent={{-254,
            -48},{-234,-28}})));
equation

  // Boundary conditions
  Extr_source.P_out = P_hot_source*1e5;
  Extr_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  Drains_source.Q_out = -Q_drains;
  Drains_source.h_out = h_drains;

    // Calibrated inputs
  Kth_cond = 61e3;
  Kth_subc = 8e3;
  Kfr_cold = 0;

  // Specification, for if "input_specs" is set to true
  reheater.S = 100;

  connect(Extr_source.C_out, reheater.C_hot_in)
    annotation (Line(points={{0,49},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, reheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(reheater.Kth_cond, Kth_cond) annotation (Line(points={{-7.8,-10},{-30,
          -10},{-30,-18}}, color={0,0,127}));
  connect(reheater.Kfr_cold, Kfr_cold) annotation (Line(points={{-18,4},{-26,4},
          {-26,16},{-36,16}}, color={0,0,127}));
  connect(reheater.Kth_subc, Kth_subc)
    annotation (Line(points={{-8,10},{-18,10},{-18,30}}, color={0,0,127}));
  connect(cold_sink.C_in, P_cold_sink_sensor.C_out)
    annotation (Line(points={{73,0},{68,0}}, color={28,108,200}));
  connect(reheater.C_cold_out, T_cold_sink_sensor.C_in)
    annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_in, T_cold_sink_sensor.C_out)
    annotation (Line(points={{48,0},{42,0}}, color={28,108,200}));
  connect(hot_sink.C_in, T_drains_sensor.C_out)
    annotation (Line(points={{0,-51},{0,-40}}, color={28,108,200}));
  connect(reheater.C_hot_out, T_drains_sensor.C_in)
    annotation (Line(points={{0,-8},{0,-20}}, color={28,108,200}));
  connect(Drains_source.C_out, reheater.C_hot_in)
    annotation (Line(points={{35,44},{0,44},{0,8}}, color={28,108,200}));
  connect(T_drains_sensor.T_sensor, T_drains)
    annotation (Line(points={{10,-30},{22,-30}}, color={0,0,127}));
  connect(T_cold_sink_sensor.T_sensor, T_cold_sink) annotation (Line(points={{
          32,10},{31,10},{31,20},{30,20}}, color={0,0,127}));
  connect(P_cold_sink_sensor.P_sensor, P_cold_sink)
    annotation (Line(points={{58,10},{58,22}}, color={0,0,127}));
end Reheater_direct;
