within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model AdmiLouvers_reverse
  "MML4 isolated AdmiLouver reverse test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  MetroscopeModelingLibrary.RefMoistAir.Pipes.AdmiLouver admiLouver(
    faulty=false)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source(
    P_0=110000,
    Q_0=50,
    T_0=293.15,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.RefMoistAir.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=50,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.RefMoistAir.TemperatureSensor source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=20,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=50,
    h_0=40100,
    T_0=293.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.RefMoistAir.FlowSensor source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=50,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=110000,
    Q_0=50,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.RefMoistAir.RelativeHumiditySensor source_humidity_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    H_start=50,
    signal_unit="%",
    display_unit="%",
    P_0=110000,
    Q_0=50,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-120,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_humidity
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-120,30})));
  MetroscopeModelingLibrary.SensorsControl.RefMoistAir.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="admiLouver_Kfr",
    show_causality=true,
    P_start=1.09,
    signal_unit="barA",
    display_unit="barA",
    P_0=109000,
    Q_0=50,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal admiLouver_Kfr(
    start=0.52)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={0,-50})));
equation
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
  connect(source_humidity_sensor.H_sensor, source_humidity)
    annotation (Line(points={{-120,10},{-120,30}}, color={0,0,127}));
  connect(source_flow_sensor.C_out, source_humidity_sensor.C_in)
    annotation (Line(points={{-150,0},{-130,0}}, color={28,108,200}));
  connect(source_humidity_sensor.C_out, admiLouver.C_in)
    annotation (Line(points={{-110,0},{-10,0}}, color={28,108,200}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(admiLouver.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{10,0},{70,0}}, color={28,108,200}));
  connect(sink_pressure_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{165,0}}, color={28,108,200}));
  connect(admiLouver_Kfr, admiLouver.Kfr)
    annotation (Line(points={{0,-50},{0,4}}, color={0,0,127}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end AdmiLouvers_reverse;
