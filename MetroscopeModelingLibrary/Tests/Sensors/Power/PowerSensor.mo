within MetroscopeModelingLibrary.Tests.Sensors.Power;
model PowerSensor
  extends Modelica.Icons.Example;

  MetroscopeModelingLibrary.Sensors.Power.PowerSensor powerSensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSource powerSource annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSink powerSink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  powerSource.W_out = -10; // Power flowing out of component is negative

  assert(abs(powerSource.W_out + powerSink.W_in) < 1e-3, "PowerSensor should not modify power on the line");
  assert(abs(powerSource.W_out + powerSensor.W_in) < 1e-3, "PowerSensor should measure the same value as the one passed to the source");
  connect(powerSensor.C_in, powerSource.C_W_out) annotation (Line(points={{-10,0},{-43.2,0}}, color={244,125,35}));
  connect(powerSensor.C_out, powerSink.C_W_in) annotation (Line(points={{9.8,0},{45,0}}, color={244,125,35}));
end PowerSensor;
