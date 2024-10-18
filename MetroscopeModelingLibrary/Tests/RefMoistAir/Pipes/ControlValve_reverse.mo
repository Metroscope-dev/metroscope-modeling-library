within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model ControlValve_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=10e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";
  input Units.Pressure sink_P(start=9e5);

  // Inputs for calibraiton
  input Real opening(start=0.5);

  // Calibrated parameters
  output Units.Cv Cvmax;

  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Pipes.ControlValve controlValve
    annotation (Placement(transformation(extent={{-10,-4},{10,18}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor opening_sensor
    annotation (Placement(transformation(extent={{-10,26},{10,46}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;
  sink.P_in = sink_P;

  // Inputs for calibration
    opening_sensor.Opening = opening;

  // Parameters
  controlValve.Cv_max = Cvmax;

  connect(source.C_out, controlValve.C_in)
    annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
  connect(sink.C_in, controlValve.C_out)
    annotation (Line(points={{39,0},{10,0}}, color={0,255,128}));
  connect(controlValve.Opening, opening_sensor.Opening)
    annotation (Line(points={{0,16},{0,25.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end ControlValve_reverse;
