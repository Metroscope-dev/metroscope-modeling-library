within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model AirCompressor_reverse
  extends Utilities.Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";

  // Parameters to calibrate
  output Real compression_rate;
  output Real eta_is;

  // Inputs for calibration
  input Real compressor_T_out(start = 406) "degC";
  input Real compressor_P_out(start = 17) "barA";

  // Initialisation parameters
  parameter Units.SpecificEnthalpy h_out_compressor_0 = 7e5; // Model won't initialize correctly without a guess value for the outlet enthalpy


  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink(h_in(start=h_out_compressor_0)) annotation (Placement(transformation(extent={{68,-10},{88,10}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor(h_out(start=h_out_compressor_0)) annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source turbine_power_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,40})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{46,-10},{66,10}})));
  MetroscopeModelingLibrary.Power.Connectors.DummyFix dummyFix annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,60})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;

  // Parameters to calibrate
  airCompressor.tau = compression_rate;
  airCompressor.eta_is = eta_is;

  connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-33,0},{-8,0}}, color={95,95,95}));
  connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{12,0},{18,0}}, color={95,95,95}));
  connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{38,0},{46,0}}, color={95,95,95}));
  connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{66,0},{73,0}}, color={95,95,95}));
  connect(airCompressor.C_W_in, turbine_power_source.C_out) annotation (Line(
      points={{12,10},{12,10},{20,10},{20,40},{73.2,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(turbine_power_source.C_out, dummyFix.W_port) annotation (Line(
      points={{73.2,40},{72,40},{60,40},{60,56}},
      color={244,125,35},
      smooth=Smooth.Bezier));
end AirCompressor_reverse;
