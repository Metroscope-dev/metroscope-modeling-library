within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model ControlValve_reverse
  extends Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

    // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Pressure sink_P(start=9e5);

  // Inputs for calibraiton
  input Real opening(start=0.9);

  // Calibrated parameters
  output Units.Cv Cvmax;


  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MetroscopeModelingLibrary.FlueGases.Pipes.ControlValve
                                                 controlValve
                                                      annotation (Placement(transformation(extent={{-10,-4},{10,16}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor opening_sensor annotation (Placement(transformation(extent={{-10,28},{10,48}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};
  sink.P_in = sink_P;

  // Inputs for calibration
  opening_sensor.Opening = opening;

  // Parameters
  controlValve.Cvmax = Cvmax;


  connect(source.C_out, controlValve.C_in) annotation (Line(points={{-23,0},{-16,0},{-16,-0.363636},{-10,-0.363636}}, color={95,95,95}));
  connect(controlValve.C_out, sink.C_in) annotation (Line(points={{10,-0.363636},{16,-0.363636},{16,0},{23,0}}, color={95,95,95}));
  connect(controlValve.Opening, opening_sensor.Opening) annotation (Line(points={{0,14.1818},{0,27.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end ControlValve_reverse;
