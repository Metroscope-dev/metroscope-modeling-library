within MetroscopeModelingLibrary.Tests.UnitTests;
package Electrical
  model TestPowerSensors
    MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source
      annotation (Placement(transformation(extent={{-92,-10},{-68,12}})));
    MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
      annotation (Placement(transformation(extent={{48,-12},{72,10}})));
    MetroscopeModelingLibrary.Electrical.Sensors.PowerSensor powerSensor
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  equation

    source.W = -1000e6;
    connect(powerSensor.C_in, source.u)
      annotation (Line(points={{-11.2,0},{-68,0}}, color={0,0,127}));
    connect(powerSensor.C_out, sink.u)
      annotation (Line(points={{11.2,0},{51.2,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TestPowerSensors;
end Electrical;
