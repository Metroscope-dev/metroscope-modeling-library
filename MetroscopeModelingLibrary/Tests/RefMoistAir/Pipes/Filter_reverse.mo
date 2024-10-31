within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model Filter_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=1e5) "Pa";
  input Utilities.Units.Temperature source_T(start=11) "degC";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

    // Parameters
  parameter Utilities.Units.Height delta_z=0;

  // Inputs for calibration
  input Real P_out(start=0.98) "barA";

  // Parameters for calibration
  output Utilities.Units.FrictionCoefficient Kfr;

  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{58,-10},
            {78,10}})));
  MetroscopeModelingLibrary.RefMoistAir.Pipes.Filter filter
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_out_sensor
    annotation (Placement(transformation(extent={{24,-10},{44,10}})));
equation
  // Boundary Conditions
  source.T_out = source_T + 273.15;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  filter.delta_z = delta_z;

  // Inputs for calibration
  P_out_sensor.P_barA = P_out;

  // Parameters for calibration
  filter.Kfr = Kfr;
  connect(source.C_out, filter.C_in)
    annotation (Line(points={{-37,0},{-10,0}}, color={0,127,127}));
  connect(sink.C_in, P_out_sensor.C_out)
    annotation (Line(points={{63,0},{44,0}}, color={0,127,127}));
  connect(P_out_sensor.C_in, filter.C_out)
    annotation (Line(points={{24,0},{16,0},{16,0},{10,0}}, color={0,127,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Filter_reverse;
