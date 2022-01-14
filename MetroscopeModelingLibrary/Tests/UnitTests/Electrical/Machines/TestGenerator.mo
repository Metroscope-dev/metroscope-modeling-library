within MetroscopeModelingLibrary.Tests.UnitTests.Electrical.Machines;
model TestGenerator
  extends Modelica.Icons.Example;

  MetroscopeModelingLibrary.Electrical.Machines.Generator generator
    annotation (Placement(transformation(extent={{-32,-18},{28,18}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{82,-10},{104,10}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-84,-10},{-62,10}})));
  MetroscopeModelingLibrary.Electrical.Sensors.PowerSensor powerSensor
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
equation

  source.W = 140; // Positive power means it is produced by the source

  // Forward
  //generator.eta = 0.998;

  // Reverse
  powerSensor.W_MW = 100; // Final electrical power

  connect(generator.C_power, source.u)
    annotation (Line(points={{-32,0},{-62,0}}, color={0,0,127}));
  connect(generator.C_elec, powerSensor.C_in)
    annotation (Line(points={{29.8,0},{48.8,0}}, color={0,0,127}));
  connect(sink.u, powerSensor.C_out)
    annotation (Line(points={{82,0},{71.2,0}}, color={0,0,127}));
end TestGenerator;
