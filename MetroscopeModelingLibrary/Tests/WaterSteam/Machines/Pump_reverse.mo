within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model Pump_reverse
  "MML4 isolated Pump reverse test"
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump(
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
  output Real pump_rm(start=0.85);
  output Real pump_VRotn(start=1400);
  output Real pump_a1(start=0);
  output Real pump_a2(start=0);
  output Real pump_b1(start=0);
  output Real pump_b2(start=0);
  output Real pump_rh_min(start=0.2);
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal pump_VRot(
    start=1400)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-40,-50})));
  output Real pump_a3(start=400);
  output Real pump_b3(start=0.8);
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="pump_a3",
    show_causality=true,
    P_start=55,
    signal_unit="barA",
    display_unit="barA",
    P_0=5500000,
    Q_0=100,
    h_0=635460)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="pump_b3",
    show_causality=true,
    T_start=151.3,
    signal_unit="degC",
    display_unit="degC",
    P_0=5500000,
    Q_0=100,
    h_0=635460,
    T_0=424.45)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,30}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,0})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source shaft_boundary
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={100,100})));
equation
  pump.rm = pump_rm;
  pump_rm = 0.85;
  pump.VRotn = pump_VRotn;
  pump_VRotn = 1400;
  pump.a1 = pump_a1;
  pump_a1 = 0;
  pump.a2 = pump_a2;
  pump_a2 = 0;
  pump.b1 = pump_b1;
  pump_b1 = 0;
  pump.b2 = pump_b2;
  pump_b2 = 0;
  pump.rh_min = pump_rh_min;
  pump_rh_min = 0.2;
  pump_VRot = 1400;
  pump.a3 = pump_a3;
  pump.b3 = pump_b3;
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
  connect(source_flow_sensor.C_out, pump.C_in)
    annotation (Line(points={{-150,0},{-20,0}}, color={28,108,200}));
  connect(pump_VRot, pump.VRot)
    annotation (Line(points={{-40,-50},{0,-50},{0,-24}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(pump.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{20,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{130,10},{130,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{90,0},{120,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{140,0},{215,0}}, color={28,108,200}));
  connect(pump.C_power, shaft_boundary.C_out)
    annotation (Line(points={{0,21.6},{104.8,21.6},{104.8,100}}, color={244,125,35}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p>Scalar API (no graphical component port): rm; Scalar API (no graphical component port): VRotn; Scalar API (no graphical component port): a1; Scalar API (no graphical component port): a2; Scalar API (no graphical component port): b1; Scalar API (no graphical component port): b2; Scalar API (no graphical component port): rh_min; Scalar API (no graphical component port): a3; Scalar API (no graphical component port): b3</p></html>"));
end Pump_reverse;
