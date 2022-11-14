within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model GasTurbine_direct
  extends Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=16e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.SpecificEnthalpy source_h(start=1.8e6) "J/kg";
  input Units.Power W_compressor(start=200e6) "W";

  // Parameters
  parameter Real compression_rate = 17;
  parameter Real eta_is = 0.9;
  parameter Real eta_mech = 0.99;


  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{28,-10},{48,10}})));
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine    gasTurbine    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_power annotation (Placement(transformation(extent={{28,30},{48,50}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.74678814,0.14086983,0.053226937,0.059115104,0.0};

  gasTurbine.W_compressor = W_compressor;

  // Parameters
  gasTurbine.tau = compression_rate;
  gasTurbine.eta_is = eta_is;
  gasTurbine.eta_mech = eta_mech;

  connect(source.C_out, gasTurbine.C_in) annotation (Line(points={{-33,0},{-8,0}}, color={95,95,95}));
  connect(gasTurbine.C_out, sink.C_in) annotation (Line(points={{12,0},{33,0}}, color={95,95,95}));
  connect(gasTurbine.C_W_out, sink_power.C_in) annotation (Line(
      points={{12,10},{12,10},{12,40},{33,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
end GasTurbine_direct;
