within MetroscopeModelingLibrary.Examples.CCGT;
model airCompressor_splitted_moistAir_flueGas_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.Temperature source_T(start=20);
  input Units.NegativeMassFlowRate source_Q(start=-600) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

  // Inputs for calibration
  input Real compressor_T_out(start = 580) "degC";
  input Real compressor_P_out(start = 20) "barA";

  // Parameters to calibrate
  output Real compression_rate;
  output Real eta_is;
  output Real compression_rate_1;


  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
  FlueGases.BoundaryConditions.Sink                              sink annotation (Placement(transformation(extent={{140,-10},{160,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor_1 annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{122,30},{102,50}})));
  Sensors.FlueGases.TemperatureSensor                             compressor_T_out_sensor annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  Sensors.FlueGases.PressureSensor                             compressor_P_out_sensor annotation (Placement(transformation(extent={{110,-10},{130,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source_fogger annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-64,44})));
  MultiFluid.Converters.RefMoistAir_to_FlueGases refMoistAir_to_FlueGases annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  FlueGases.Machines.AirCompressor airCompressor_2 annotation (Placement(transformation(extent={{40,-8},{60,8}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T + 273.15;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Fogger
  source_fogger.Q_out = - 1e-3; // Calibration without fogging
  source_fogger.T_out = 20 + 273.15;
  source_fogger.Xi_out = {0.99};

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;

  // Parameters to calibrate
  compression_rate = airCompressor_2.P_out/airCompressor_1.P_in;
  airCompressor_1.eta_is = eta_is;
  airCompressor_2.eta_is = eta_is;
  airCompressor_1.tau = compression_rate_1;
  compression_rate_1 = compression_rate/2;


  connect(source_fogger.C_out, airCompressor_1.C_in) annotation (Line(
      points={{-64,39},{-64,0},{-40,0}},
      color={0,255,128},
      thickness=0.5));
  connect(source.C_out, airCompressor_1.C_in) annotation (Line(
      points={{-79,0},{-40,0}},
      color={0,255,128},
      thickness=1));
  connect(airCompressor_1.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-20,40},{107.2,40}}, color={244,125,35}));
  connect(airCompressor_1.C_out, refMoistAir_to_FlueGases.inlet) annotation (Line(
      points={{-20,0},{0,0}},
      color={0,255,128},
      thickness=1));
  connect(refMoistAir_to_FlueGases.outlet, airCompressor_2.C_in) annotation (Line(
      points={{20,0},{40,0}},
      color={95,95,95},
      thickness=1));
  connect(airCompressor_2.C_W_in, power_source.C_out) annotation (Line(points={{60,6},{60,40},{107.2,40}}, color={244,125,35}));
  connect(airCompressor_2.C_out, compressor_T_out_sensor.C_in) annotation (Line(
      points={{60,0},{80,0}},
      color={95,95,95},
      thickness=1));
  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(
      points={{100,0},{110,0}},
      color={95,95,95},
      thickness=1));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(
      points={{130,0},{145,0}},
      color={95,95,95},
      thickness=1));
  annotation (Diagram(coordinateSystem(extent={{-100,-40},{180,80}}),
                      graphics={Text(
          extent={{-78,68},{-48,54}},
          textColor={0,0,0},
          textString="Fogging Xi_out = {0.99}
Equivalent to 99% water")}), Icon(coordinateSystem(extent={{-100,-40},{180,80}})));
end airCompressor_splitted_moistAir_flueGas_reverse;
