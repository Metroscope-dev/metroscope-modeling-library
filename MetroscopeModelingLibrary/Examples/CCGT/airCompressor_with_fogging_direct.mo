within MetroscopeModelingLibrary.Examples.CCGT;
model airCompressor_with_fogging_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.Temperature source_T(start=20);
  input Units.NegativeMassFlowRate source_Q(start=-600) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

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

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{22,30},{2,50}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{10,-10},{30,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source_fogger annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-64,44})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T + 273.15;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Fogger
  source_fogger.Q_out = -2.5;
  source_fogger.T_out = source_T + 273.15;
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
  connect(source_fogger.C_out, airCompressor.C_in) annotation (Line(points={{-64,39},{-64,0},{-40,0}}, color={0,255,128},
      thickness=0.5));
  connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-40,0}}, color={0,255,128},
      thickness=1));
  connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-20,40},{7.2,40}}, color={244,125,35}));
  connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-20,0},{10,0}}, color={0,255,128},
      thickness=1));
  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{30,0},{40,0}}, color={0,255,128},
      thickness=1));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{60,0},{79,0}}, color={0,255,128},
      thickness=1));
  annotation (Diagram(graphics={Text(
          extent={{-56,52},{-26,38}},
          textColor={0,0,0},
          textString="Fogging Xi_out = {0.99}
Equivalent to 99% water")}));
end airCompressor_with_fogging_direct;
