within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model ControlValve_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Pressure sink_P(start=9e5) "Pa";

  // Inputs for calibration
  input Real opening(start=0.35);

  // Calibrated parameter
  output Units.Cv Cvmax "Cvmax";

  // Components
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-6.10623e-16})));

  WaterSteam.Pipes.ControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor opening_sensor annotation (Placement(transformation(extent={{-10,50},{10,70}})));
equation
  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  sink.P_in = sink_P;

  // Inputs for calibration
  opening_sensor.Opening = opening;

  // Calibrated Parameters
  control_valve.Cvmax = Cvmax;

  connect(control_valve.C_out, sink.C_in) annotation (Line(points={{16.5,-1.81818e-06},{34.75,-1.81818e-06},{34.75,0},{53,0}}, color={28,108,200}));
  connect(control_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}},color={28,108,200}));
  connect(control_valve.Opening, opening_sensor.Opening) annotation (Line(points={{0,23.7575},{0,49.8}}, color={0,0,127}));
end ControlValve_reverse;
