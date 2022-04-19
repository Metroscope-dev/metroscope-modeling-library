within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model GasTurbine_reverse
  extends Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=16e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.SpecificEnthalpy source_h(start=1.8e6) "J/kg";
  input Units.Power Wcompressor(start=200e6) "W";

  // Parameters
  parameter Real eta_mech = 0.99;

  // Inputs for calibration
  input Real turbine_P_out(start=1) "barA";
  input Real turbine_T_out(start=600) "degC";

  // Parameters for calibration
  output Real compression_rate;
  output Real eta_is;

  // Parameters for initialization
  parameter Units.SpecificEnthalpy turbine_h_out_0 = 1e6;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink(h_in(start=turbine_h_out_0)) annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine    gasTurbine(h_out(start=turbine_h_out_0))    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_power annotation (Placement(transformation(extent={{28,30},{48,50}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_compressor_power annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-38,40})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlueGasesPressureSensor turbine_P_out_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlueGasesTemperatureSensor turbine_T_out_sensor annotation (Placement(transformation(extent={{50,-10},{70,10}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.74678814,0.14086983,0.053226937,0.059115104,0.0};

  sink_compressor_power.W_in = Wcompressor;

  // Parameters
  gasTurbine.eta_is = eta_is;
  gasTurbine.eta_mech = eta_mech;

  // Inputs for calibration
  turbine_P_out_sensor.P_barA = turbine_P_out;
  turbine_T_out_sensor.T_degC = turbine_T_out;

  // Parameters for calibration
  gasTurbine.tau = compression_rate;

  connect(source.C_out, gasTurbine.C_in) annotation (Line(points={{-33,0},{-8,0}}, color={95,95,95}));
  connect(gasTurbine.C_W_out, sink_power.C_in) annotation (Line(
      points={{12,10},{12,10},{12,40},{33,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(sink_compressor_power.C_in, gasTurbine.C_W_compressor) annotation (Line(
      points={{-33,40},{-8,40},{-8,10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(gasTurbine.C_out, turbine_P_out_sensor.C_in) annotation (Line(points={{12,0},{22,0}}, color={95,95,95}));
  connect(turbine_P_out_sensor.C_out, turbine_T_out_sensor.C_in) annotation (Line(points={{42,0},{50,0}}, color={95,95,95}));
  connect(turbine_T_out_sensor.C_out, sink.C_in) annotation (Line(points={{70,0},{79,0}}, color={95,95,95}));
end GasTurbine_reverse;
