within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model AirCompressor_direct


  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-104,
            -10},{-84,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{112,-10},
            {132,10}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{10,-10},
            {30,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source turbine_power_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={38,40})));
  SensorsControl.FlueGases.TemperatureSensor ambiant_temp_sensor(
    sensor_function="BC",
    T_start=15,
    signal_unit="degC")
    annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
  SensorsControl.FlueGases.PressureSensor ambiant_pressure_sensor(
    sensor_function="BC",
    P_start=1,
    signal_unit="barA")
    annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  SensorsControl.FlueGases.FlowSensor air_flow_sensor(
    sensor_function="BC",
    Q_start=500,
    signal_unit="kg/s")
    annotation (Placement(transformation(extent={{-26,-10},{-6,10}})));
  SensorsControl.FlueGases.TemperatureSensor discharge_temp_sensor(
    sensor_function="Calibration",
    causality="airCompressor_eta_is",
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  SensorsControl.FlueGases.PressureSensor discharge_pressure_sensor(
    sensor_function="Calibration",
    causality="airCompressor_tau",
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  Utilities.Interfaces.BoundaryCondition ambiant_temp annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-74,26}), iconTransformation(extent={{-78,0},{-38,40}})));
  Utilities.Interfaces.BoundaryCondition ambiant_pressure annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-44,26}), iconTransformation(extent={{-78,0},{-38,40}})));
  Utilities.Interfaces.BoundaryCondition air_flow annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-16,26}), iconTransformation(extent={{-78,0},{-38,40}})));
  Utilities.Interfaces.Observable discharge_temp annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={48,20}), iconTransformation(extent={{-122,-52},{-102,-32}})));
  Utilities.Interfaces.Observable discharge_pressure annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={90,22}), iconTransformation(extent={{-122,-52},{-102,-32}})));
  Utilities.Interfaces.GenericReal airCompressor_tau annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={14,-20}), iconTransformation(extent={{18,-38},{38,-18}})));
  Utilities.Interfaces.GenericReal airCompressor_eta_is annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={30,-18}), iconTransformation(extent={{18,-38},{38,-18}})));
equation

  airCompressor_tau = 15;
  airCompressor_eta_is = 0.95;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  connect(source.C_out, ambiant_temp_sensor.C_in)
    annotation (Line(points={{-89,0},{-84,0}}, color={95,95,95}));
  connect(ambiant_temp_sensor.C_out, ambiant_pressure_sensor.C_in)
    annotation (Line(points={{-64,0},{-54,0}}, color={95,95,95}));
  connect(ambiant_pressure_sensor.C_out, air_flow_sensor.C_in)
    annotation (Line(points={{-34,0},{-26,0}}, color={95,95,95}));
  connect(air_flow_sensor.C_out, airCompressor.C_in)
    annotation (Line(points={{-6,0},{10,0}}, color={95,95,95}));
  connect(airCompressor.C_out, discharge_temp_sensor.C_in)
    annotation (Line(points={{30,0},{38,0}}, color={95,95,95}));
  connect(discharge_temp_sensor.C_out, discharge_pressure_sensor.C_in)
    annotation (Line(points={{58,0},{80,0}}, color={95,95,95}));
  connect(discharge_pressure_sensor.C_out, sink.C_in)
    annotation (Line(points={{100,0},{117,0}}, color={95,95,95}));
  connect(ambiant_temp_sensor.T_sensor, ambiant_temp)
    annotation (Line(points={{-74,10},{-74,26}}, color={0,0,127}));
  connect(ambiant_pressure_sensor.P_sensor, ambiant_pressure)
    annotation (Line(points={{-44,10},{-44,26}}, color={0,0,127}));
  connect(air_flow_sensor.Q_sensor, air_flow)
    annotation (Line(points={{-16,10},{-16,26}}, color={0,0,127}));
  connect(discharge_temp_sensor.T_sensor, discharge_temp)
    annotation (Line(points={{48,10},{48,20}}, color={0,0,127}));
  connect(discharge_pressure_sensor.P_sensor, discharge_pressure)
    annotation (Line(points={{90,10},{90,22}}, color={0,0,127}));
  connect(airCompressor.tau,airCompressor_tau)
    annotation (Line(points={{14,-7.5},{14,-20}}, color={0,0,127}));
  connect(airCompressor.eta_is,airCompressor_eta_is)
    annotation (Line(points={{20,-6.25},{20,-18},{30,-18}}, color={0,0,127}));
  connect(turbine_power_source.C_out, airCompressor.C_W_in) annotation (Line(
        points={{33.2,40},{24,40},{24,7.5},{30,7.5}}, color={244,125,35}));
end AirCompressor_direct;
