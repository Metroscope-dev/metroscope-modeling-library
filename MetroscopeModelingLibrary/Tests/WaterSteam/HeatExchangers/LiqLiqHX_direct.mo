within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_direct

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Utilities.Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Utilities.Units.MassFlowRate Q_cold_source(start=100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-100,
            -10},{-80,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{72,-10},
            {92,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX( input_specs = true)
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-72})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-42,18},{-34,26}}), iconTransformation(extent={{-224,42},{-204,
            62}})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-20,22},{-12,30}}), iconTransformation(extent={{-216,42},{-196,
            62}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
          extent={{12,18},{20,26}}), iconTransformation(extent={{-218,52},{-198,
            72}})));
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
public
  Sensors_Control.WaterSteam.PressureSensor P_cold_in_sensor(
    sensor_function="BC",
    P_start=20,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-56,-6},{-42,8}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_in_sensor(
    sensor_function="BC",
    T_start=50,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-76,-6},{-62,8}})));
  Utilities.Interfaces.BoundaryCondition T_cold_in annotation (Placement(
        transformation(extent={{-76,14},{-68,22}}), iconTransformation(extent={{
            -184,-48},{-144,-8}})));
  Utilities.Interfaces.BoundaryCondition P_cold_in annotation (Placement(
        transformation(extent={{-60,16},{-52,24}}), iconTransformation(extent={{
            -184,-48},{-144,-8}})));
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
  Utilities.Interfaces.BoundaryCondition T_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={24,44}),  iconTransformation(extent={{-184,-48},{-144,-8}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=90,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,-53})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
    sensor_function="BC",
    T_start=55,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{34,-6},{48,8}})));
  Utilities.Interfaces.BoundaryCondition T_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={40,18}), iconTransformation(extent={{-184,-48},{-144,-8}})));
  Utilities.Interfaces.RealOutput T_hot_out annotation (Placement(
        transformation(extent={{16,-58},{24,-50}}), iconTransformation(extent={{
            -218,52},{-198,72}})));
  Utilities.Interfaces.RealOutput P_hot_out annotation (Placement(
        transformation(extent={{16,-36},{24,-28}}), iconTransformation(extent={{
            -218,52},{-198,72}})));
  Utilities.Interfaces.RealOutput P_cold_out annotation (Placement(
        transformation(extent={{58,16},{66,24}}), iconTransformation(extent={{-218,
            52},{-198,72}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=19,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{56,-6},{70,8}})));
equation
  //hot_source.Q_out = - Q_hot_source;
  cold_source.Q_out = - Q_cold_source;

  // Calibrated parameters
  Kth = 492;
  Kfr_cold = 1000;
  Kfr_hot = 40000;

  // Specifications, for if "input_specs" is set to true
  liqLiqHX.S = 100;

  connect(liqLiqHX.Kfr_cold, Kfr_cold) annotation (Line(points={{-18,3.2},{-30,3.2},
          {-30,22},{-38,22}}, color={0,0,127}));
  connect(liqLiqHX.Kth, Kth)
    annotation (Line(points={{-9.6,10},{-16,10},{-16,26}}, color={0,0,127}));
  connect(Kfr_hot, liqLiqHX.Kfr_hot) annotation (Line(points={{16,22},{16,10},{4.8,
          10}},                  color={0,0,127}));
  connect(cold_sink.C_in, P_cold_out_sensor.C_out)
    annotation (Line(points={{77,0},{70,0},{70,1}}, color={28,108,200}));
  connect(Kfr_hot, Kfr_hot)
    annotation (Line(points={{16,22},{16,22}}, color={0,0,127}));
  connect(liqLiqHX.C_hot_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{0,-8},{0,-24},{1,-24}}, color={28,108,200}));
  connect(liqLiqHX.C_cold_in, P_cold_in_sensor.C_out) annotation (Line(points={{
          -16.2,0},{-38,0},{-38,1},{-42,1}}, color={28,108,200}));
  connect(P_cold_in_sensor.C_in, T_cold_in_sensor.C_out)
    annotation (Line(points={{-56,1},{-62,1}}, color={28,108,200}));
  connect(T_cold_in_sensor.C_in, cold_source.C_out)
    annotation (Line(points={{-76,1},{-76,0},{-85,0}}, color={28,108,200}));
  connect(T_cold_in_sensor.T_sensor, T_cold_in) annotation (Line(points={{-69,8},
          {-68,8},{-68,12},{-72,12},{-72,18}}, color={0,0,127}));
  connect(P_cold_in_sensor.P_sensor, P_cold_in) annotation (Line(points={{-49,8},
          {-50.5,8},{-50.5,20},{-56,20}}, color={0,0,127}));
  connect(hot_source.C_out, P_hot_in_sensor.C_in)
    annotation (Line(points={{0,85},{0,72},{1,72}}, color={28,108,200}));
  connect(P_hot_in_sensor.P_sensor, P_hot_in)
    annotation (Line(points={{8,65},{8,64},{18,64}}, color={0,0,127}));
  connect(P_hot_in_sensor.C_out, T_hot_in_sensor.C_in) annotation (Line(points={{1,58},{
          0,58},{0,50},{1,50}},         color={28,108,200}));
  connect(liqLiqHX.C_hot_in, T_hot_in_sensor.C_out)
    annotation (Line(points={{0,8},{0,36},{1,36}}, color={28,108,200}));
  connect(T_hot_in_sensor.T_sensor, T_hot_in) annotation (Line(points={{8,43},{10,
          43},{10,44},{24,44}}, color={0,0,127}));
  connect(hot_sink.C_in, T_hot_out_sensor.C_out)
    annotation (Line(points={{0,-67},{1,-66},{1,-60}}, color={28,108,200}));
  connect(T_hot_out_sensor.C_in, P_hot_out_sensor.C_out)
    annotation (Line(points={{1,-46},{1,-38}}, color={28,108,200}));
  connect(T_cold_out_sensor.C_out, P_cold_out_sensor.C_in)
    annotation (Line(points={{48,1},{56,1}}, color={28,108,200}));
  connect(T_cold_out_sensor.C_in, liqLiqHX.C_cold_out) annotation (Line(points={
          {34,1},{20,1},{20,0},{16,0}}, color={28,108,200}));
  connect(T_cold_out, T_cold_out_sensor.T_sensor)
    annotation (Line(points={{40,18},{41,18},{41,8}}, color={28,108,200}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out) annotation (Line(points={{8,-53},
          {8,-54.5},{20,-54.5},{20,-54}}, color={0,0,127}));
  connect(P_hot_out_sensor.P_sensor, P_hot_out)
    annotation (Line(points={{8,-31},{8,-32},{20,-32}}, color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out) annotation (Line(points={{63,8},
          {63,10},{62,10},{62,20}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_direct;
