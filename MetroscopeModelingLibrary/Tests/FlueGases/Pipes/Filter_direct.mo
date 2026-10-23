within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Filter_direct
  "MML4 isolated Filter direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;
  MetroscopeModelingLibrary.FlueGases.Pipes.Filter filter(
    faulty=false)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source(
    P_0=110000,
    Q_0=50,
    T_0=293.15,
    h_0=322465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=50,
    h_0=322465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=20,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=50,
    h_0=322465,
    T_0=293.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=50,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=110000,
    Q_0=50,
    h_0=322465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="filter_Kfr",
    show_causality=true,
    P_start=1.09,
    signal_unit="barA",
    display_unit="barA",
    P_0=109000,
    Q_0=50,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal filter_Kfr(
    start=0.52)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={0,-50})));
  Utilities.Interfaces.Observable y annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={4,114}), iconTransformation(extent={{2,98},{22,118}})));
  SensorsControl.FlueGases.DeltaPressureSensor deltaPressureSensor
    annotation (Placement(transformation(extent={{-12,52},{8,72}})));
equation
  source.Xi_out = {0.768,0.232,0,0,0};
  filter_Kfr = 0.52;
  connect(source_pressure_sensor.P_sensor, source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(source_temperature_sensor.T_sensor, source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(source_flow_sensor.Q_sensor, source_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(filter_Kfr, filter.Kfr)
    annotation (Line(points={{0,-50},{0,8}}, color={0,0,127}));
  connect(sink.C_in, sink_pressure_sensor.C_out)
    annotation (Line(points={{165,0},{90,0}}, color={95,95,95}));
  connect(sink_pressure_sensor.C_in, filter.C_out)
    annotation (Line(points={{70,0},{20,0}}, color={95,95,95}));
  connect(filter.C_in, source_flow_sensor.C_out)
    annotation (Line(points={{-20,0},{-150,0}}, color={95,95,95}));
  connect(source_pressure_sensor.C_out, source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={95,95,95}));
  connect(source_pressure_sensor.C_in, source.C_out)
    annotation (Line(points={{-250,0},{-275,0}}, color={95,95,95}));
  connect(source_temperature_sensor.C_out, source_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={95,95,95}));
  connect(deltaPressureSensor.C_in, source_flow_sensor.C_out) annotation (Line(
        points={{-12,62},{-80,62},{-80,0},{-150,0}}, color={95,95,95}));
  connect(deltaPressureSensor.C_out, filter.C_out) annotation (Line(points={{8,
          62},{26,62},{26,70},{46,70},{46,0},{20,0}}, color={95,95,95}));
  connect(y, deltaPressureSensor.DP_sensor) annotation (Line(points={{4,114},{4,
          80},{-4,80},{-4,72},{-2,72}}, color={0,0,127}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Filter_direct;
