within MetroscopeModelingLibrary.Tests.Multifluid.Machines;
model AirCompressor_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.Temperature source_T(start=20);
  input Units.NegativeMassFlowRate source_Q(start=-600) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

  // Inputs for calibration
  output Real compressor_T_out(start = 580) "degC";
  output Real compressor_P_out(start = 20) "barA";

  // Hypothesis
  output Real tau_moist;

  // Parameters to calibrate
  parameter Real tau = 20;
  parameter Real eta_is = 0.72272354;

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{14,30},{-6,50}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MultiFluid.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T + 273.15;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;

  // Parameters to calibrate
  tau = airCompressor.tau;
  eta_is = airCompressor.eta_is;

  // Hypothesis
  tau_moist = tau/4;
  tau_moist = airCompressor.tau_moist;

  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(
      points={{8,0},{18,0}},
      color={95,95,95},
      thickness=1));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(
      points={{38,0},{53,0}},
      color={95,95,95},
      thickness=1));
  connect(source.C_out, airCompressor.inlet) annotation (Line(points={{-59,0},{-40,0}}, color={0,255,128}));
  connect(airCompressor.outlet, compressor_T_out_sensor.C_in) annotation (Line(points={{-20,0},{-12,0}}, color={95,95,95}));
  connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-16,6},{-16,40},{-0.8,40}}, color={244,125,35}));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
                             Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
end AirCompressor_direct;
