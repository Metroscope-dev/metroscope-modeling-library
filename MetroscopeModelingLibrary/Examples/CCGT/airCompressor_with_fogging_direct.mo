within MetroscopeModelingLibrary.Examples.CCGT;
model airCompressor_with_fogging_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  inner parameter Boolean show_causality = true "true to show causality, false to hide it";
  inner parameter Boolean display_output = true "Used to switch ON or OFF output display";

  // Boundary conditinos
  input Real source_P(start=1) "bar";
  input Units.Temperature source_T(start=20) "degC";
  input Units.MassFlowRate source_Q(start=600) "kg/s";
  input Real source_relative_humidity(start=50) "%";
  input Units.MassFlowRate Q_fogging(start=2.5) "kg/s";
  input Units.Temperature T_fogging(start=20) "degC";

  // Inputs for calibration
  output Real compressor_T_out(start = 450) "degC";
  output Real compressor_P_out(start = 15) "barA";

  // Parameters to calibrate
  parameter Real compression_rate = 15;
  parameter Real eta_is = 0.76828927;

  // Outputs of interest
  RefMoistAirMedium.ThermodynamicState state_fogger;
  output Real x_liq_water_fogger;
  output Real Q_fogger;

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-234,-10},{-214,10}})));
  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-32,-8},{-12,8}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{30,30},{10,50}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source_fogger annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,84})));
  Sensors.RefMoistAir.TemperatureSensor T_fogging_sensor(sensor_function="BC") annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-80,60})));
  Sensors.RefMoistAir.TemperatureSensor ambient_T_sensor(sensor_function="BC") annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  Sensors.RefMoistAir.TemperatureSensor compressor_T_in_sensor annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Sensors.RefMoistAir.FlowSensor Q_fogging_sensor(sensor_function="BC") annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-80,30})));
  Sensors.RefMoistAir.FlowSensor Q_air_sensor(sensor_function="BC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,0})));
  Sensors.RefMoistAir.PressureSensor ambient_P_sensor(sensor_function="BC")
                                                      annotation (Placement(transformation(extent={{-180,-10},{-160,10}})));
  Sensors.RefMoistAir.RelativeHumiditySensor ambient_RH_sensor(sensor_function="BC")
                                                               annotation (Placement(transformation(extent={{-210,-10},{-190,10}})));
equation
  // Boundary conditions
  ambient_P_sensor.P_barA = source_P;
  ambient_T_sensor.T_degC = source_T;
  Q_air_sensor.Q = source_Q;
  ambient_RH_sensor.relative_humidity_pc = source_relative_humidity;

  // Fogger
  Q_fogging_sensor.Q = Q_fogging;
  T_fogging_sensor.T_degC = T_fogging;
  source_fogger.Xi_out = {0.99};

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;

  // Parameters to calibrate
  airCompressor.tau = compression_rate;
  airCompressor.eta_is = eta_is;

  // Outputs of ineterst
  state_fogger = RefMoistAirMedium.setState_pTX(source_fogger.P_out, source_fogger.T_out, source_fogger.Xi_out);
  x_liq_water_fogger = RefMoistAirMedium.massFractionWaterNonVapor(state_fogger);
  Q_fogger = -x_liq_water_fogger*source_fogger.Q_out;
  connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-12,6},{-12,40},{15.2,40}},color={244,125,35}));
  connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-12,0},{8,0}},  color={0,255,128},
      thickness=1));
  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{28,0},{38,0}}, color={0,255,128},
      thickness=1));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{58,0},{67,0}}, color={0,255,128},
      thickness=1));
  connect(source_fogger.C_out,T_fogging_sensor. C_in) annotation (Line(
      points={{-80,79},{-80,70}},
      color={0,255,128},
      thickness=1));
  connect(compressor_T_in_sensor.C_out, airCompressor.C_in) annotation (Line(
      points={{-40,0},{-32,0}},
      color={0,255,128},
      thickness=1));
  connect(T_fogging_sensor.C_out, Q_fogging_sensor.C_in) annotation (Line(
      points={{-80,50},{-80,40}},
      color={0,255,128},
      thickness=1));
  connect(Q_fogging_sensor.C_out, compressor_T_in_sensor.C_in) annotation (Line(
      points={{-80,20},{-80,0},{-60,0}},
      color={0,255,128},
      thickness=1));
  connect(ambient_T_sensor.C_out, Q_air_sensor.C_in) annotation (Line(
      points={{-130,0},{-120,0}},
      color={0,255,128},
      thickness=1));
  connect(Q_air_sensor.C_out, compressor_T_in_sensor.C_in) annotation (Line(
      points={{-100,0},{-60,0}},
      color={0,255,128},
      thickness=1));
  connect(ambient_P_sensor.C_out, ambient_T_sensor.C_in) annotation (Line(
      points={{-160,0},{-150,0}},
      color={0,255,128},
      thickness=1));
  connect(source.C_out, ambient_RH_sensor.C_in) annotation (Line(
      points={{-219,0},{-210,0}},
      color={0,255,128},
      thickness=1));
  connect(ambient_RH_sensor.C_out, ambient_P_sensor.C_in) annotation (Line(
      points={{-190,0},{-180,0}},
      color={0,255,128},
      thickness=1));
  annotation (Diagram(coordinateSystem(extent={{-240,-40},{100,100}}),
                      graphics={Text(
          extent={{-64,94},{-34,80}},
          textColor={0,0,0},
          textString="Fogging Xi_out = {0.99}
Equivalent to 99% water")}), Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
end airCompressor_with_fogging_direct;
