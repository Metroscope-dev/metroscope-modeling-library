within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model Pipe_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=10e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

    // Parameters
  parameter Utilities.Units.Height delta_z=1;

  // Inputs for calibration
  input Real P_out(start=9) "barA";

  // Parameters for calibration
  output Utilities.Units.FrictionCoefficient Kfr;

  MetroscopeModelingLibrary.RefMoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{24,-10},{44,10}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  pipe.delta_z = delta_z;

  // Inputs for calibration
  P_out_sensor.P_barA = P_out;

  // Parameters for calibration
  pipe.Kfr = Kfr;

  connect(source.C_out, pipe.C_in) annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
  connect(pipe.C_out, P_out_sensor.C_in) annotation (Line(points={{10,0},{24,0},{24,0}}, color={0,255,128}));
  connect(P_out_sensor.C_out, sink.C_in) annotation (Line(points={{44,0},{55,0}}, color={0,255,128}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_reverse;
