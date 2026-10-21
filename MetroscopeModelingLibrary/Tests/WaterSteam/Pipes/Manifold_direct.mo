within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Manifold_direct
  "MML4 isolated Manifold direct test"
  MetroscopeModelingLibrary.WaterSteam.Pipes.Manifold manifold
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source1(
    P_0=2000000,
    Q_0=50,
    T_0=373.15,
    h_0=420000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor source1_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=20,
    signal_unit="barA",
    display_unit="barA",
    P_0=2000000,
    Q_0=50,
    h_0=420000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source1_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor source1_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=100,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=50,
    h_0=420000,
    T_0=373.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source1_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor source1_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=50,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2000000,
    Q_0=50,
    h_0=420000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source1_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source2(
    P_0=2000000,
    Q_0=50,
    T_0=353.15,
    h_0=336000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,180})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor source2_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=80,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=50,
    h_0=336000,
    T_0=353.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source2_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,210}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor source2_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=50,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2000000,
    Q_0=50,
    h_0=336000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source2_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,210}), iconTransformation(extent={{-140,40},{-100,80}})));
  input MetroscopeModelingLibrary.Utilities.Units.DifferentialPressure balance_pressure(start=0);
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor sink_temperature_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=90,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=100,
    h_0=378000,
    T_0=363.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
equation
  manifold.DP_input = balance_pressure;
  connect(source1_pressure_sensor.P_sensor, source1_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(source1.C_out, source1_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(source1_temperature_sensor.T_sensor, source1_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(source1_pressure_sensor.C_out, source1_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(source1_flow_sensor.Q_sensor, source1_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(source1_temperature_sensor.C_out, source1_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={28,108,200}));
  connect(source1_flow_sensor.C_out, manifold.C_in_1)
    annotation (Line(points={{-150,0},{-10,0}}, color={28,108,200}));
  connect(source2_temperature_sensor.T_sensor, source2_temperature)
    annotation (Line(points={{-240,190},{-240,210}}, color={0,0,127}));
  connect(source2.C_out, source2_temperature_sensor.C_in)
    annotation (Line(points={{-275,180},{-250,180}}, color={28,108,200}));
  connect(source2_flow_sensor.Q_sensor, source2_flow)
    annotation (Line(points={{-200,190},{-200,210}}, color={0,0,127}));
  connect(source2_temperature_sensor.C_out, source2_flow_sensor.C_in)
    annotation (Line(points={{-230,180},{-210,180}}, color={28,108,200}));
  connect(source2_flow_sensor.C_out, manifold.C_in_2)
    annotation (Line(points={{-190,180},{0,180},{0,10}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(manifold.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{10,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{165,0}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Manifold_direct;
