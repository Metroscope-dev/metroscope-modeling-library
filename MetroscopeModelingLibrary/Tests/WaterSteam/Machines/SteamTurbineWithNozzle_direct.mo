within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model SteamTurbineWithNozzle_direct
  "MML4 isolated SteamTurbineWithNozzle direct test"
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbineWithNozzle steamTurbineWithNozzle(
    area_nz=1,
    P_in_0=6000000,
    P_out_0=1000000,
    Q_0=100,
    T_in_0=773.15,
    T_out_0=623.15,
    h_in_0=3400000,
    h_out_0=3000000)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source(
    P_0=6000000,
    Q_0=100,
    T_0=773.15,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=60,
    signal_unit="barA",
    display_unit="barA",
    P_0=6000000,
    Q_0=100,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=500,
    signal_unit="degC",
    display_unit="degC",
    P_0=6000000,
    Q_0=100,
    h_0=3400000,
    T_0=773.15)
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
    P_0=6000000,
    Q_0=100,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamTurbineWithNozzle_Cst(
    start=4000000)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-40,-60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamTurbineWithNozzle_eta_is(
    start=0.85)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={20,-60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamTurbineWithNozzle_eta_nz(
    start=0.95)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={65,-60})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="steamTurbineWithNozzle_Cst",
    show_causality=true,
    P_start=10,
    signal_unit="barA",
    display_unit="barA",
    P_0=1000000,
    Q_0=100,
    h_0=1260000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="steamTurbineWithNozzle_eta_is",
    show_causality=true,
    T_start=300,
    signal_unit="degC",
    display_unit="degC",
    P_0=1000000,
    Q_0=100,
    h_0=1260000,
    T_0=573.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,30}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,0})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink shaft_boundary
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={100,100})));
equation
  steamTurbineWithNozzle_Cst = 4000000;
  steamTurbineWithNozzle_eta_is = 0.85;
  steamTurbineWithNozzle_eta_nz = 0.95;
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
  connect(source_flow_sensor.C_out, steamTurbineWithNozzle.C_in)
    annotation (Line(points={{-150,0},{-40,0}}, color={28,108,200}));
  connect(steamTurbineWithNozzle_Cst, steamTurbineWithNozzle.Cst)
    annotation (Line(points={{-40,-60},{-26.4,-60},{-26.4,30.4}}, color={0,0,127}));
  connect(steamTurbineWithNozzle_eta_is, steamTurbineWithNozzle.eta_is)
    annotation (Line(points={{20,-60},{-2.8,-60},{-2.8,34.8}}, color={0,0,127}));
  connect(steamTurbineWithNozzle_eta_nz, steamTurbineWithNozzle.eta_nz)
    annotation (Line(points={{65,-60},{30.8,-60},{30.8,-27.6}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(steamTurbineWithNozzle.C_out, sink_pressure_sensor.C_in)
    annotation (Line(points={{40,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{130,10},{130,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{90,0},{120,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{140,0},{215,0}}, color={28,108,200}));
  connect(steamTurbineWithNozzle.C_W_out, shaft_boundary.C_in)
    annotation (Line(points={{24,32},{95,32},{95,100}}, color={244,125,35}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end SteamTurbineWithNozzle_direct;
