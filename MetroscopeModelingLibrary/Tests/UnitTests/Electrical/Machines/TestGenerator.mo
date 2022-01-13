within MetroscopeModelingLibrary.Tests.UnitTests.Electrical.Machines;
model TestGenerator
  extends Modelica.Icons.Example;
  input Real Wmech(start=140); // power produced by a component is considered positive, contrary to the flow convention

  // Forward causality
  input Real eta(start=99.8);
  output Real Welec;

  // Reverse causality
  //input Real Welec(start=140);
  //output Real eta;

  MetroscopeModelingLibrary.Electrical.Machines.Generator generator
    annotation (Placement(transformation(extent={{-32,-18},{28,18}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{64,-10},{86,10}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-84,-10},{-62,10}})));
equation
  source.W = Wmech;

  generator.eta = eta;
  generator.Welec = Welec;

  connect(generator.C_elec, sink.u)
    annotation (Line(points={{29.8,0},{64,0}}, color={0,0,127}));
  connect(generator.C_power, source.u)
    annotation (Line(points={{-32,0},{-62,0}}, color={0,0,127}));
end TestGenerator;
