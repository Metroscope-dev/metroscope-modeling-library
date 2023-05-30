within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model AirCompressor_direct
  extends Utilities.Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

  // Parameters
  parameter Real compression_rate = 17;
  parameter Real eta_is = 0.9;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{28,-10},{48,10}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source turbine_power_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,40})));
  MetroscopeModelingLibrary.Power.Connectors.DummyFix dummy_fix annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={60,62})));
equation

  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  airCompressor.tau = compression_rate;
  airCompressor.eta_is = eta_is;


  connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-33,0},{-8,0}}, color={95,95,95}));
  connect(airCompressor.C_out, sink.C_in) annotation (Line(points={{12,0},{33,0}}, color={95,95,95}));
  connect(airCompressor.C_W_in, turbine_power_source.C_out) annotation (Line(
      points={{12,10},{20,10},{20,40},{73.2,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(dummy_fix.W_port, turbine_power_source.C_out) annotation (Line(
      points={{60,58},{60,40},{73.2,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
end AirCompressor_direct;
