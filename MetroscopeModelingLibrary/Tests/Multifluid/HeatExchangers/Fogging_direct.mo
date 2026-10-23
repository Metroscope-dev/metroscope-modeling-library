within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Fogging_direct
  "MML4 isolated Fogging direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Fogging fogging
    annotation (Placement(transformation(extent={{-30,-30},{30,30}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source gas_source(
    P_0=110000,
    Q_0=660,
    T_0=301.95,
    h_0=332145)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor gas_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=660,
    h_0=332145)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition gas_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor gas_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=28.8,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=660,
    h_0=332145,
    T_0=301.95)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition gas_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor gas_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=660,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=110000,
    Q_0=660,
    h_0=332145)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition gas_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source water_source(
    P_0=500000,
    Q_0=2.5,
    T_0=293.01,
    h_0=83412)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,180})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor water_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=5,
    signal_unit="barA",
    display_unit="barA",
    P_0=500000,
    Q_0=2.5,
    h_0=83412)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition water_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,210}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor water_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=19.86,
    signal_unit="degC",
    display_unit="degC",
    P_0=500000,
    Q_0=2.5,
    h_0=83412,
    T_0=293.01)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition water_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,210}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor water_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=2.5,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=500000,
    Q_0=2.5,
    h_0=83412)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition water_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,210}), iconTransformation(extent={{-140,-5},{-100,35}})));
  output Real fogging_Q_fg(start=660);
  output Real fogging_x_vapor(start=1);
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="fogging_x_vapor",
    show_causality=true,
    T_start=23.5,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=662.5,
    h_0=500000,
    T_0=296.65)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,0})));
equation
  gas_source.Xi_out = {0.768,0.232,0,0,0};
  fogging.Q_fg = fogging_Q_fg;
  fogging_Q_fg = 660;
  fogging.x_vapor = fogging_x_vapor;
  fogging_x_vapor = 1;
  connect(gas_source_pressure_sensor.P_sensor, gas_source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(gas_source.C_out, gas_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(gas_source_temperature_sensor.T_sensor, gas_source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(gas_source_pressure_sensor.C_out, gas_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(gas_source_flow_sensor.Q_sensor, gas_source_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(gas_source_temperature_sensor.C_out, gas_source_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={28,108,200}));
  connect(gas_source_flow_sensor.C_out, fogging.C_fg_inlet)
    annotation (Line(points={{-150,0},{-30,0}}, color={28,108,200}));
  connect(water_source_pressure_sensor.P_sensor, water_source_pressure)
    annotation (Line(points={{-240,190},{-240,210}}, color={0,0,127}));
  connect(water_source.C_out, water_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,180},{-250,180}}, color={28,108,200}));
  connect(water_source_temperature_sensor.T_sensor, water_source_temperature)
    annotation (Line(points={{-200,190},{-200,210}}, color={0,0,127}));
  connect(water_source_pressure_sensor.C_out, water_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,180},{-210,180}}, color={28,108,200}));
  connect(water_source_flow_sensor.Q_sensor, water_source_flow)
    annotation (Line(points={{-160,190},{-160,210}}, color={0,0,127}));
  connect(water_source_temperature_sensor.C_out, water_source_flow_sensor.C_in)
    annotation (Line(points={{-190,180},{-170,180}}, color={28,108,200}));
  connect(water_source_flow_sensor.C_out, fogging.C_water_in)
    annotation (Line(points={{-150,180},{0,180},{0,18}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(fogging.C_fg_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{30,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{165,0}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p>Scalar API (no graphical component port): Q_fg; Scalar API (no graphical component port): x_vapor</p></html>"));
end Fogging_direct;
