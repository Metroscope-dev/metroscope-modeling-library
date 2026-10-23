within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model SteamExtractionSplitter_direct
  "MML4 isolated SteamExtractionSplitter direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source(
    P_0=264000,
    Q_0=1000,
    T_0=400,
    h_0=2650000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=2.64,
    signal_unit="barA",
    display_unit="barA",
    P_0=264000,
    Q_0=1000,
    h_0=2650000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=1000,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=264000,
    Q_0=1000,
    h_0=2650000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  input MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy source_enthalpy(start=2650000);
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamExtractionSplitter_alpha(
    start=0.9)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={40,60})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink main_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={120,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor extraction_sink_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=100,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=264000,
    Q_0=100,
    h_0=546000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition extraction_sink_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink extraction_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,-120})));
  output MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy main_enthalpy(start=2670000);
equation
  source.h_out = source_enthalpy;
  steamExtractionSplitter_alpha = 0.9;
  main_enthalpy = steamExtractionSplitter.main_flow.h_out;
  connect(source_pressure_sensor.P_sensor, source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(source.C_out, source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(source_flow_sensor.Q_sensor, source_flow)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(source_pressure_sensor.C_out, source_flow_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(source_flow_sensor.C_out, steamExtractionSplitter.C_in)
    annotation (Line(points={{-190,0},{-21.2,0}}, color={28,108,200}));
  connect(steamExtractionSplitter_alpha, steamExtractionSplitter.alpha)
    annotation (Line(points={{40,60},{16,60},{16,4.8}}, color={0,0,127}));
  connect(steamExtractionSplitter.C_main_out, main_sink.C_in)
    annotation (Line(points={{21.2,0},{115,0}}, color={28,108,200}));
  connect(extraction_sink_flow_sensor.Q_sensor, extraction_sink_flow)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(steamExtractionSplitter.C_ext_out, extraction_sink_flow_sensor.C_in)
    annotation (Line(points={{0,-13.6},{70,-13.6},{70,-120}}, color={28,108,200}));
  connect(extraction_sink_flow_sensor.C_out, extraction_sink.C_in)
    annotation (Line(points={{90,-120},{165,-120}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end SteamExtractionSplitter_direct;
