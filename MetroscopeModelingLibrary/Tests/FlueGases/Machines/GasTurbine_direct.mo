within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model GasTurbine_direct
  "MML4 isolated GasTurbine direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine gasTurbine(
    faulty=false,
    P_in_0=1500000,
    P_out_0=100000,
    Q_0=500,
    T_in_0=1473.15,
    T_out_0=1323.15,
    h_in_0=1600000,
    h_out_0=1000000)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source(
    P_0=1500000,
    Q_0=500,
    T_0=1473.15,
    h_0=1620465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=15,
    signal_unit="barA",
    display_unit="barA",
    P_0=1500000,
    Q_0=500,
    h_0=1620465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=1200,
    signal_unit="degC",
    display_unit="degC",
    P_0=1500000,
    Q_0=500,
    h_0=1620465,
    T_0=1473.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=500,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=1500000,
    Q_0=500,
    h_0=1620465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal gasTurbine_eta_is(
    start=0.9)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={0,-60})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor sink_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1,
    signal_unit="barA",
    display_unit="barA",
    P_0=100000,
    Q_0=500,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="gasTurbine_eta_is",
    show_causality=true,
    T_start=600,
    signal_unit="degC",
    display_unit="degC",
    P_0=100000,
    Q_0=500,
    h_0=500000,
    T_0=873.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,0})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink shaft_boundary
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={100,100})));
equation
  source.Xi_out = {0.768,0.232,0,0,0};
  gasTurbine_eta_is = 0.9;
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
  connect(source_flow_sensor.C_out, gasTurbine.C_in)
    annotation (Line(points={{-150,0},{-40,0}}, color={28,108,200}));
  connect(gasTurbine_eta_is, gasTurbine.eta_is)
    annotation (Line(points={{0,-60},{-8,-60},{-8,-32}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(gasTurbine.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{40,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{130,10},{130,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{90,0},{120,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{140,0},{215,0}}, color={28,108,200}));
  connect(gasTurbine.C_W_shaft, shaft_boundary.C_in)
    annotation (Line(points={{40,40},{95,40},{95,100}}, color={244,125,35}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end GasTurbine_direct;
