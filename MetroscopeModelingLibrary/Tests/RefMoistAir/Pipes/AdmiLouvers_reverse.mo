within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model AdmiLouvers_reverse
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
  MetroscopeModelingLibrary.RefMoistAir.Pipes.AdmiLouver
                                                admiLouver
                                                     annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
equation
    // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  admiLouver.delta_z = delta_z;

  // Inputs for calibration
  P_out_sensor.P_barA = P_out;

  // Parameters for calibration
  admiLouver.Kfr = Kfr;
  connect(P_out_sensor.C_out, sink.C_in) annotation (Line(points={{42,0},{53,0}}, color={0,255,128}));
  connect(admiLouver.C_out, P_out_sensor.C_in) annotation (Line(points={{8,0},{22,0}}, color={0,255,128}));
  connect(source.C_out, admiLouver.C_in) annotation (Line(points={{-39,0},{-12,0}}, color={0,255,128}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end AdmiLouvers_reverse;
