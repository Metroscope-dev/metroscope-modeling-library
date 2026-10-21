within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model FixedSpeedPump_direct
  "MML4 isolated FixedSpeedPump direct test"
  MetroscopeModelingLibrary.WaterSteam.Machines.FixedSpeedPump fixedSpeedPump(
    P_in_0=2000000,
    P_out_0=5500000,
    Q_0=100,
    T_in_0=423.15,
    T_out_0=424.15,
    h_in_0=630000,
    h_out_0=635000)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source(
    P_0=2000000,
    Q_0=100,
    T_0=423.15,
    h_0=630000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=20,
    signal_unit="barA",
    display_unit="barA",
    P_0=2000000,
    Q_0=100,
    h_0=630000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=150,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=100,
    h_0=630000,
    T_0=423.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=100,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2000000,
    Q_0=100,
    h_0=630000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal fixedSpeedPump_hn(
    start=400)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-50,-40})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal fixedSpeedPump_rh(
    start=0.8)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={50,-40})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="fixedSpeedPump_hn",
    show_causality=true,
    P_start=55,
    signal_unit="barA",
    display_unit="barA",
    P_0=5500000,
    Q_0=100,
    h_0=635460)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="fixedSpeedPump_rh",
    show_causality=true,
    T_start=151.3,
    signal_unit="degC",
    display_unit="degC",
    P_0=5500000,
    Q_0=100,
    h_0=635460,
    T_0=424.45)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,30}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,0})));
equation
  fixedSpeedPump_hn = 400;
  fixedSpeedPump_rh = 0.8;
  connect(source_pressure_sensor.P_sensor, source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(source.C_out, source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(source_temperature_sensor.T_sensor, source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(source_pressure_sensor.C_out, source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(source_flow_sensor.Q_sensor, source_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(source_temperature_sensor.C_out, source_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={28,108,200}));
  connect(source_flow_sensor.C_out, fixedSpeedPump.C_in)
    annotation (Line(points={{-150,0},{-20,0}}, color={28,108,200}));
  connect(fixedSpeedPump_hn, fixedSpeedPump.hn)
    annotation (Line(points={{-50,-40},{-12,-40},{-12,-16}}, color={0,0,127}));
  connect(fixedSpeedPump_rh, fixedSpeedPump.rh)
    annotation (Line(points={{50,-40},{12,-40},{12,-16}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(fixedSpeedPump.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{20,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{130,10},{130,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{90,0},{120,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{140,0},{215,0}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end FixedSpeedPump_direct;
