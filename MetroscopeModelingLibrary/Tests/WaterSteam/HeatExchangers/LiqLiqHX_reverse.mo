within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Utilities.Units.MassFlowRate Q_cold_source(start=100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-100,
            -10},{-80,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{72,-10},
            {92,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX(
      input_specs=true)
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-72})));
  Utilities.Interfaces.RealOutput Cond_Kth_subc annotation (Placement(
        transformation(extent={{-20,22},{-12,30}}), iconTransformation(extent={{
            -216,42},{-196,62}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
          extent={{12,18},{20,26}}), iconTransformation(extent={{-218,52},{-198,
            72}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
    sensor_function="BC",
    T_start=55,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{36,-6},{50,8}})));
  Sensors_Control.WaterSteam.PressureSensor P_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kfr hot",
    P_start=49,
    signal_unit="barA",
    display_unit="barA")                                     annotation (
      Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,-31})));
  Utilities.Interfaces.CalibrationInput P_hot_out annotation (Placement(
        transformation(extent={{16,-36},{24,-28}}), iconTransformation(extent={{
            -192,-48},{-152,-8}})));
  Utilities.Interfaces.CalibrationInput T_hot_out annotation (Placement(
        transformation(extent={{14,-54},{22,-46}}), iconTransformation(extent={{
            -192,-48},{-152,-8}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_in_sensor(
    sensor_function="BC",
    T_start=50,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-76,-6},{-62,8}})));
  Utilities.Interfaces.BoundaryCondition T_cold_in annotation (Placement(
        transformation(extent={{-76,14},{-68,22}}), iconTransformation(extent={{
            -184,-48},{-144,-8}})));
public
  Sensors_Control.WaterSteam.PressureSensor P_hot_in_sensor(
    sensor_function="BC",
    P_start=50,
    signal_unit="barA",
    display_unit="barA") annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,65})));
  Utilities.Interfaces.BoundaryCondition P_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={18,64}),  iconTransformation(extent={{-184,-48},{-144,-8}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_hot_in_sensor(
    sensor_function="BC",
    T_start=100,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,43})));
  Sensors_Control.WaterSteam.TemperatureSensor T_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=90,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,-51})));
  Utilities.Interfaces.RealOutput T_hot_in
                                          annotation (Placement(transformation(
          extent={{14,38},{22,46}}), iconTransformation(extent={{-218,52},{-198,
            72}})));
  Utilities.Interfaces.RealExpression Cond_Kfr_cold_subc
    annotation (Placement(transformation(extent={{-48,8},{-28,28}})));
  Utilities.Interfaces.BoundaryCondition T_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={50,28}), iconTransformation(extent={{-184,-48},{-144,-8}})));
equation

  // Boundary conditions
  cold_source.Q_out = - Q_cold_source;
  cold_source.P_out = 20*6894.75729;
  hot_source.T_out = 100+273.15;

  // Specifications, for if "input_specs" is set to true
  liqLiqHX.S = 100;

  connect(liqLiqHX.Kth, Cond_Kth_subc)
    annotation (Line(points={{-9.6,10},{-16,10},{-16,26}}, color={0,0,127}));
  connect(Kfr_hot, liqLiqHX.Kfr_hot) annotation (Line(points={{16,22},{16,10},{4.8,
          10}},                  color={0,0,127}));
  connect(liqLiqHX.C_cold_out, T_cold_out_sensor.C_in)
    annotation (Line(points={{16,0},{16,1},{36,1}}, color={28,108,200}));
  connect(Kfr_hot, Kfr_hot)
    annotation (Line(points={{16,22},{16,22}}, color={0,0,127}));
  connect(liqLiqHX.C_hot_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{0,-8},{0,-24},{1,-24}}, color={28,108,200}));
  connect(P_hot_out, P_hot_out_sensor.P_sensor)
    annotation (Line(points={{20,-32},{20,-31},{8,-31}}, color={28,108,200}));
  connect(T_cold_in_sensor.C_in, cold_source.C_out)
    annotation (Line(points={{-76,1},{-76,0},{-85,0}}, color={28,108,200}));
  connect(T_cold_in_sensor.T_sensor, T_cold_in) annotation (Line(points={{-69,8},
          {-68,8},{-68,12},{-72,12},{-72,18}}, color={0,0,127}));
  connect(hot_source.C_out, P_hot_in_sensor.C_in)
    annotation (Line(points={{0,85},{0,72},{1,72}}, color={28,108,200}));
  connect(P_hot_in_sensor.P_sensor, P_hot_in)
    annotation (Line(points={{8,65},{8,64},{18,64}}, color={0,0,127}));
  connect(P_hot_in_sensor.C_out, T_hot_in_sensor.C_in) annotation (Line(points={
          {1,58},{2,58},{2,50},{1,50}}, color={28,108,200}));
  connect(liqLiqHX.C_hot_in, T_hot_in_sensor.C_out)
    annotation (Line(points={{0,8},{0,36},{1,36}}, color={28,108,200}));
  connect(hot_sink.C_in, T_hot_out_sensor.C_out)
    annotation (Line(points={{0,-67},{1,-66},{1,-58}}, color={28,108,200}));
  connect(T_hot_out_sensor.C_in, P_hot_out_sensor.C_out)
    annotation (Line(points={{1,-44},{1,-38}}, color={28,108,200}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out)
    annotation (Line(points={{8,-51},{8,-50},{18,-50}}, color={0,0,127}));
  connect(T_hot_in_sensor.T_sensor,T_hot_in)
    annotation (Line(points={{8,43},{8,42},{18,42}}, color={0,0,127}));
  connect(cold_sink.C_in, T_cold_out_sensor.C_out) annotation (Line(points={{77,
          0},{54,0},{54,1},{50,1}}, color={28,108,200}));
  connect(liqLiqHX.Kfr_cold, Cond_Kfr_cold_subc.y)
    annotation (Line(points={{-18,3.2},{-38,3.2},{-38,13}}, color={0,0,127}));
  connect(T_cold_in_sensor.C_out, liqLiqHX.C_cold_in) annotation (Line(points={{
          -62,1},{-26,1},{-26,0},{-16.2,0}}, color={28,108,200}));
  connect(T_cold_out, T_cold_out_sensor.T_sensor) annotation (Line(points={{50,28},
          {50,12},{43,12},{43,8}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_reverse;
