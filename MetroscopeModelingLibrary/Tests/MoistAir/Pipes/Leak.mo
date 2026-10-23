within MetroscopeModelingLibrary.Tests.MoistAir.Pipes;
model Leak
  "MML4 isolated Leak test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MoistAirTestIcon;
  MetroscopeModelingLibrary.MoistAir.Pipes.Leak leak
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source(
    P_0=110000,
    Q_0=10,
    T_0=293.15,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=20,
    h_0=40100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.TemperatureSensor source_temperature_sensor(
    sensor_function="Unidentified",
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
  Utilities.Interfaces.Observable                                  source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.RelativeHumiditySensor source_humidity_sensor(
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
  MetroscopeModelingLibrary.SensorsControl.MoistAir.PressureSensor sink_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=50,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  Utilities.Interfaces.BoundaryCondition                    sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
  input MetroscopeModelingLibrary.Utilities.Units.DifferentialPressure balance_pressure(start=0);
equation
  leak.Q = 10;
  source.h_out = 40100;
  connect(source_pressure_sensor.P_sensor, source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(source.C_out, source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(source_temperature_sensor.T_sensor, source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(source_pressure_sensor.C_out, source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(source_humidity_sensor.H_sensor, source_humidity)
    annotation (Line(points={{-120,10},{-120,30}}, color={0,0,127}));
  connect(source_humidity_sensor.C_out, leak.C_in)
    annotation (Line(points={{-110,0},{-10,0}}, color={28,108,200}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(leak.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{10,0},{70,0}}, color={28,108,200}));
  connect(sink_pressure_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{165,0}}, color={28,108,200}));
  connect(source_temperature_sensor.C_out, source_humidity_sensor.C_in)
    annotation (Line(points={{-190,0},{-130,0}}, color={85,170,255}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Leak;
