within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestDPSensor
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterDeltaPressureSensor
    waterDeltaPressureSensor
    annotation (Placement(transformation(extent={{-10,12},{10,32}})));
equation

  source.Q_out = -1500;
  source.T_out = 273.15 + 20;
  source.P_out = 1e5;

  sink.h_vol = 1e6;

  singularPressureLoss.Kfr = 1;

  connect(source.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{-20,0},{-10,0}}, color={63,81,181}));
  connect(sink.C_in, singularPressureLoss.C_out)
    annotation (Line(points={{22,0},{10.2,0}},color={63,81,181}));
  connect(source.C_out, waterDeltaPressureSensor.C_in) annotation (Line(points={
          {-20,0},{-20,22.2},{-10,22.2}}, color={63,81,181}));
  connect(waterDeltaPressureSensor.C_out, singularPressureLoss.C_out)
    annotation (Line(points={{10,22},{16,22},{16,0},{10.2,0}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},
            {40,40}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-20},{40,40}})));
end TestDPSensor;
