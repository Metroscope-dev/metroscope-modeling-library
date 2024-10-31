within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model AdmiLouvers_direct
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      // Boundary conditions
  input Utilities.Units.Pressure source_P(start=10e5) "Pa";
  input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

    // Parameters
  parameter Utilities.Units.FrictionCoefficient Kfr=100;
  parameter Utilities.Units.Height delta_z=0;

  MetroscopeModelingLibrary.RefMoistAir.Pipes.AdmiLouver
                                                admiLouver
                                                     annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  // Parameters
  admiLouver.Kfr = Kfr;
  admiLouver.delta_z = delta_z;
  connect(source.C_out, admiLouver.C_in) annotation (Line(points={{-31,0},{-10,0}}, color={0,127,127}));
  connect(admiLouver.C_out, sink.C_in) annotation (Line(points={{10,0},{45,0}}, color={0,127,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end AdmiLouvers_direct;
