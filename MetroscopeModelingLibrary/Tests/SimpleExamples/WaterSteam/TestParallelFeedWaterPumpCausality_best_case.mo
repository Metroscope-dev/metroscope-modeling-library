within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model TestParallelFeedWaterPumpCausality_best_case
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;

  // BC
  input Real STs_P_in(start=60) "barA";
  input Real STs_Q_in(start=60) "barA";
  input Real pumps_P_in(start=60) "barA";
  input Real pumps_Q_in(start=60) "barA";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source STs_source
    annotation (Placement(transformation(extent={{-122,42},{-102,62}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink STs_sink
    annotation (Placement(transformation(extent={{96,42},{116,62}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source pumps_source
    annotation (Placement(transformation(extent={{118,-58},{98,-38}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink pumps_sink
    annotation (Placement(transformation(extent={{-100,-58},{-120,-38}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor STs_Q_in_sensor
    annotation (Placement(transformation(extent={{-70,42},{-50,62}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    STs_P_in_sensor
    annotation (Placement(transformation(extent={{-96,42},{-76,62}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor
    pumps_Q_in_sensor
    annotation (Placement(transformation(extent={{60,-58},{40,-38}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    pumps_P_in_sensor
    annotation (Placement(transformation(extent={{88,-58},{68,-38}})));
equation
  // STs
  STs_source.h_out = 1e6;
  STs_sink.h_vol = 1e6;
  STs_Q_in_sensor = STs_Q_in;
  STs_P_in_sensor = STs_P_in;

  // Pumps
  pumps_source.h_out = 1e6;
  pumps_sink.h_vol = 1e6;
  pumps_Q_in_sensor = pumps_Q_in;
  pumps_P_in_sensor = pumps_P_in;
  connect(STs_Q_in_sensor.C_out, STs_sink.C_in)
    annotation (Line(points={{-49.8,52},{96,52}}, color={63,81,181}));
  connect(STs_source.C_out, STs_P_in_sensor.C_in)
    annotation (Line(points={{-102,52},{-96,52}}, color={63,81,181}));
  connect(STs_P_in_sensor.C_out, STs_Q_in_sensor.C_in)
    annotation (Line(points={{-75.8,52},{-70,52}}, color={63,81,181}));
  connect(pumps_Q_in_sensor.C_out, pumps_sink.C_in)
    annotation (Line(points={{39.8,-48},{-100,-48}}, color={63,81,181}));
  connect(pumps_source.C_out, pumps_P_in_sensor.C_in)
    annotation (Line(points={{98,-48},{88,-48}}, color={63,81,181}));
  connect(pumps_P_in_sensor.C_out,pumps_Q_in_sensor. C_in)
    annotation (Line(points={{67.8,-48},{60,-48}}, color={63,81,181}));
  annotation (Diagram(coordinateSystem(extent={{-200,-100},{240,100}})), Icon(
        coordinateSystem(extent={{-200,-100},{240,100}})));
end TestParallelFeedWaterPumpCausality_best_case;
