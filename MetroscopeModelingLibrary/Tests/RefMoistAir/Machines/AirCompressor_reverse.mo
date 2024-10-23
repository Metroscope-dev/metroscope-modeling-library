within MetroscopeModelingLibrary.Tests.RefMoistAir.Machines;
model AirCompressor_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.Temperature source_T(start=293.15);
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

  // Inputs for calibration
  input Real compressor_T_out(start = 406) "degC";
  input Real compressor_P_out(start = 17) "barA";

  // Parameters to calibrate
  output Real compression_rate;
  output Real eta_is;

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{22,30},{2,50}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{10,-10},{30,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;

  // Parameters to calibrate
  airCompressor.tau = compression_rate;
  airCompressor.eta_is = eta_is;

  connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-40,0}}, color={0,255,128}));
  connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-20,40},{7.2,40}}, color={244,125,35}));
  connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-20,0},{10,0}}, color={0,255,128}));
  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{30,0},{40,0}}, color={0,255,128}));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{60,0},{79,0}}, color={0,255,128}));
end AirCompressor_reverse;
