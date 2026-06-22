within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model DryReheater_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;


  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Utilities.Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";


  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dryReheater(
      input_specs=true)
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-36})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-20,18},{-12,26}}), iconTransformation(extent={{-32,6},{-12,
            26}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-36,8},{-28,16}}), iconTransformation(extent={{-34,2},{-14,22}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_sink_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=49,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_sink_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=70,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  Utilities.Interfaces.CalibrationInput P_cold_sink annotation (Placement(
        transformation(extent={{28,20},{36,28}}), iconTransformation(extent={{-254,
            -44},{-214,-4}})));
  Utilities.Interfaces.CalibrationInput T_cold_sink annotation (Placement(
        transformation(extent={{50,20},{58,28}}), iconTransformation(extent={{-254,
            -44},{-214,-4}})));
equation

  // Boundary conditions
  hot_source.P_out = P_hot_source*1e5;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  // Specifications, for when "input_specs" is activated in component
  dryReheater.S = 100;

  connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, dryReheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(dryReheater.C_hot_out, hot_sink.C_in) annotation (Line(points={{0,-8},
          {0,-19.5},{8.88178e-16,-19.5},{8.88178e-16,-31}}, color={28,108,200}));
  connect(dryReheater.Kth, Kth)
    annotation (Line(points={{-8,10},{-16,10},{-16,22}}, color={0,0,127}));
  connect(dryReheater.Kfr_cold, Kfr_cold) annotation (Line(points={{-18,4},{-24,
          4},{-24,12},{-32,12}}, color={0,0,127}));
  connect(dryReheater.C_cold_out, P_cold_sink_sensor.C_in)
    annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_out, T_cold_sink_sensor.C_in)
    annotation (Line(points={{42,0},{48,0}}, color={28,108,200}));
  connect(cold_sink.C_in, T_cold_sink_sensor.C_out)
    annotation (Line(points={{73,0},{68,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.P_sensor, P_cold_sink) annotation (Line(points={{32,10},
          {32,24}},                     color={0,0,127}));
  connect(T_cold_sink_sensor.T_sensor, T_cold_sink)
    annotation (Line(points={{58,10},{58,24},{54,24}}, color={0,0,127}));
end DryReheater_reverse;
