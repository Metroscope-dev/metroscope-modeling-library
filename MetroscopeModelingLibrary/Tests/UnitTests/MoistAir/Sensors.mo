within MetroscopeModelingLibrary.Tests.UnitTests.MoistAir;
package Sensors
  extends Modelica.Icons.ExamplesPackage;

  model TestMoistAirSensors
    extends Modelica.Icons.Example;
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source AirSource
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink
      annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    MetroscopeModelingLibrary.MoistAir.Sensors.MoistAirTemperatureSensor
      moistAirTemperatureSensor
      annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
    MetroscopeModelingLibrary.MoistAir.Sensors.MoistAirFlowSensor
      moistAirFlowSensor
      annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
    MetroscopeModelingLibrary.MoistAir.Sensors.MoistAirPressureSensor
      moistAirPressureSensor
      annotation (Placement(transformation(extent={{6,-10},{26,10}})));
    MetroscopeModelingLibrary.MoistAir.Sensors.MoistAirHumiditySensor
      moistAirHumiditySensor
      annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  equation

    AirSource.P_out = 5e5;
    AirSource.T_vol = 24+273.15;
    AirSource.relative_humidity = 0.01;
    AirSource.Q_out = -1;

    sink.h_vol = 1e5;
    sink.Xi_vol[1] = 0.01;

    connect(AirSource.C_out, moistAirTemperatureSensor.C_in)
      annotation (Line(points={{-60,0},{-52,0}}, color={63,81,181}));
    connect(moistAirTemperatureSensor.C_out, moistAirFlowSensor.C_in)
      annotation (Line(points={{-31.8,0},{-24,0}}, color={63,81,181}));
    connect(moistAirFlowSensor.C_out, moistAirPressureSensor.C_in)
      annotation (Line(points={{-3.8,0},{6,0}}, color={63,81,181}));
    connect(moistAirPressureSensor.C_out, moistAirHumiditySensor.C_in)
      annotation (Line(points={{26.2,0},{34,0}}, color={63,81,181}));
    connect(sink.C_in, moistAirHumiditySensor.C_out)
      annotation (Line(points={{60,0},{54.2,0}}, color={63,81,181}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TestMoistAirSensors;
end Sensors;
