within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Desuperheating_direct
  "MML4 isolated Desuperheating direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;
  MetroscopeModelingLibrary.WaterSteam.Pipes.Desuperheating desuperheating
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source steam_source(
    P_0=12000000,
    Q_0=80,
    T_0=848.15,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor steam_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=120,
    signal_unit="barA",
    display_unit="barA",
    P_0=12000000,
    Q_0=80,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition steam_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor steam_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=575,
    signal_unit="degC",
    display_unit="degC",
    P_0=12000000,
    Q_0=80,
    h_0=3400000,
    T_0=848.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition steam_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor steam_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=80,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=12000000,
    Q_0=80,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition steam_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source water_source(
    P_0=13000000,
    Q_0=2,
    T_0=353.15,
    h_0=336000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,180})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor water_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=130,
    signal_unit="barA",
    display_unit="barA",
    P_0=13000000,
    Q_0=2,
    h_0=336000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition water_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,210}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor water_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=80,
    signal_unit="degC",
    display_unit="degC",
    P_0=13000000,
    Q_0=2,
    h_0=336000,
    T_0=353.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition water_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,210}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal desuperheating_T_setpoint(
    start=560)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,70})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor sink_temperature_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=560,
    signal_unit="degC",
    display_unit="degC",
    P_0=12000000,
    Q_0=82,
    h_0=2352000,
    T_0=833.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
equation
  desuperheating_T_setpoint = 560;
  connect(steam_source_pressure_sensor.P_sensor, steam_source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(steam_source.C_out, steam_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(steam_source_temperature_sensor.T_sensor, steam_source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(steam_source_pressure_sensor.C_out, steam_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(steam_source_flow_sensor.Q_sensor, steam_source_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(steam_source_temperature_sensor.C_out, steam_source_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={28,108,200}));
  connect(steam_source_flow_sensor.C_out, desuperheating.C_in)
    annotation (Line(points={{-150,0},{-10,0}}, color={28,108,200}));
  connect(water_source_pressure_sensor.P_sensor, water_source_pressure)
    annotation (Line(points={{-240,190},{-240,210}}, color={0,0,127}));
  connect(water_source.C_out, water_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,180},{-250,180}}, color={28,108,200}));
  connect(water_source_temperature_sensor.T_sensor, water_source_temperature)
    annotation (Line(points={{-200,190},{-200,210}}, color={0,0,127}));
  connect(water_source_pressure_sensor.C_out, water_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,180},{-210,180}}, color={28,108,200}));
  connect(water_source_temperature_sensor.C_out, desuperheating.Water_in)
    annotation (Line(points={{-190,180},{-7,180},{-7,3}}, color={28,108,200}));
  connect(desuperheating_T_setpoint, desuperheating.T_setpoint)
    annotation (Line(points={{-30,70},{6,70},{6,3.4}}, color={0,0,127}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(desuperheating.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{10,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{165,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, desuperheating.T_measured)
    annotation (Line(points={{80,10},{6,10},{6,-3.4}}, color={0,0,127}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Desuperheating_direct;
