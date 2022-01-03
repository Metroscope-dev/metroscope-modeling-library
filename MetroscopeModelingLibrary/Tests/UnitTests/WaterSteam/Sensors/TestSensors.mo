within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestSensors

  input Real Q(start=50); // Flow rate of source in tons per hour
  input Real T(start=100); // Temperature of source in degC
  input Real P(start=10); // Pressure of source in bar
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor waterFlowSensor
    annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor
    waterTemperatureSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    waterPressureSensor
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation

  waterFlowSensor.Q_th = Q;  // inject in sensor with the right unit
  waterTemperatureSensor.T_degC = T; // inject in sensor with the right unit
  waterPressureSensor.P_barA = P; // inject in sensor with the right unit

  sink.h_vol = 1e6;

  connect(waterFlowSensor.C_in, source.C_out)
    annotation (Line(points={{-46,0},{-60,0}}, color={63,81,181}));
  connect(waterPressureSensor.C_out, sink.C_in)
    annotation (Line(points={{46.2,0},{60,0}}, color={63,81,181}));
  connect(waterFlowSensor.C_out, waterTemperatureSensor.C_in)
    annotation (Line(points={{-25.8,0},{-10,0}}, color={63,81,181}));
  connect(waterTemperatureSensor.C_out, waterPressureSensor.C_in)
    annotation (Line(points={{10.2,0},{26,0}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},
            {80,20}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-80,-20},{80,20}})));
end TestSensors;
