within MetroscopeModelingLibrary.Tests.MoistAir.Pipes;
model Pipe_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MoistAirTestIcon;
      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=10e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";


  // Inputs for calibration
  input Real P_out(start=9) "barA";



  MetroscopeModelingLibrary.MoistAir.Pipes.FrictionPipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  Utilities.Interfaces.Observable Kfr annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,22}), iconTransformation(extent={{-124,-34},{-104,-14}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;


  // Inputs for calibration
  P_out_sensor.P_barA = P_out;


  connect(pipe.C_in,source. C_out) annotation (Line(points={{-10,0},{-37,0}}, color={85,170,255}));
  connect(pipe.C_out, P_out_sensor.C_in) annotation (Line(points={{10,0},{24,0}}, color={85,170,255}));
  connect(sink.C_in, P_out_sensor.C_out) annotation (Line(points={{55,0},{44,0}}, color={85,170,255}));
  connect(pipe.Kfr, Kfr) annotation (Line(points={{0,4},{0,22}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe_reverse;
