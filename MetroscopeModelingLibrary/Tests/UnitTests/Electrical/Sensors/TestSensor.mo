within MetroscopeModelingLibrary.Tests.UnitTests.Electrical.Sensors;
model TestSensor "UnitTest for power sensor"
  extends Modelica.Icons.Example;
  input Real Welec(start=1.0); // power produced by a component is considered positive, contrary to the flow convention


  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{64,-10},{86,10}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-84,-10},{-62,10}})));
  MetroscopeModelingLibrary.Electrical.Sensors.PowerSensor powerSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  // Forward
  source.W = Welec;

  // Reverse
  //powerSensor.W = Welec;

  connect(sink.u, powerSensor.C_out)
    annotation (Line(points={{64,0},{11.2,0}}, color={0,0,127}));
  connect(source.u, powerSensor.C_in)
    annotation (Line(points={{-62,0},{-11.2,0}}, color={0,0,127}));
end TestSensor;
