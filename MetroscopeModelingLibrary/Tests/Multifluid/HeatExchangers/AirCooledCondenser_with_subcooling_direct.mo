within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_with_subcooling_direct
  "MML4 isolated AirCooledCondenser_with_subcooling direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.AirCooledCondenser_with_subcooling airCooledCondenser_with_subcooling(
    faulty=false,
    S_cond=150000,
    S_subc=10000,
    Kfr_hot=0,
    C_incond=0,
    P_offset=0,
    Q_cold_0=2300,
    Q_hot_0=21,
    T_hot_out_0=363.15,
    h_cold_in_0=35075,
    h_cold_out_0=50000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(
    P_0=100200,
    Q_0=2300,
    T_0=288.15,
    h_0=35075)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,-120})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.PressureSensor cold_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.002,
    signal_unit="barA",
    display_unit="barA",
    P_0=100200,
    Q_0=2300,
    h_0=35075)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-90}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.TemperatureSensor cold_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=15,
    signal_unit="degC",
    display_unit="degC",
    P_0=100200,
    Q_0=2300,
    h_0=35075,
    T_0=288.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-90}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.FlowSensor cold_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=2300,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=100200,
    Q_0=2300,
    h_0=35075)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,-90}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.RelativeHumiditySensor cold_source_humidity_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    H_start=50,
    signal_unit="%",
    display_unit="%",
    P_0=100200,
    Q_0=2300,
    h_0=35075)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-120,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_humidity
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-120,-90})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source(
    P_0=91000,
    Q_0=21,
    T_0=370,
    h_0=2400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,160})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor hot_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=21,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=91000,
    Q_0=21,
    h_0=2400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,190}), iconTransformation(extent={{-140,85},{-100,125}})));
  input MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy hot_source_enthalpy(start=2400000);
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor condensing_pressure_sensor(
    sensor_function="Calibration",
    causality="airCooledCondenser_with_subcooling_Kth_cond",
    show_causality=true,
    P_start=0.91,
    signal_unit="barA",
    display_unit="barA",
    P_0=91000,
    Q_0=21,
    h_0=2400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-80,90})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable condensing_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-80,120}), iconTransformation(extent={{100,220},{140,260}})));
  output MetroscopeModelingLibrary.Utilities.Units.HeatExchangeCoefficient airCooledCondenser_with_subcooling_Kth_cond(start=9);
  output MetroscopeModelingLibrary.Utilities.Units.HeatExchangeCoefficient airCooledCondenser_with_subcooling_Kth_subc(start=2);
  MetroscopeModelingLibrary.SensorsControl.MoistAir.TemperatureSensor cold_sink_temperature_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=25,
    signal_unit="degC",
    display_unit="degC",
    P_0=100200,
    Q_0=2300,
    h_0=50000,
    T_0=298.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cold_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,-120})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor hot_sink_temperature_sensor(
    sensor_function="Calibration",
    causality="airCooledCondenser_with_subcooling_Kth_subc",
    show_causality=true,
    T_start=90,
    signal_unit="degC",
    display_unit="degC",
    P_0=91000,
    Q_0=21,
    h_0=378000,
    T_0=363.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable hot_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,190}), iconTransformation(extent={{100,130},{140,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,160})));
equation
  hot_source.h_out = hot_source_enthalpy;
  airCooledCondenser_with_subcooling.Kth_cond = airCooledCondenser_with_subcooling_Kth_cond;
  airCooledCondenser_with_subcooling_Kth_cond = 9;
  airCooledCondenser_with_subcooling.Kth_subc = airCooledCondenser_with_subcooling_Kth_subc;
  airCooledCondenser_with_subcooling_Kth_subc = 2;
  connect(cold_source_pressure_sensor.P_sensor, cold_source_pressure)
    annotation (Line(points={{-240,-110},{-240,-90}}, color={0,0,127}));
  connect(cold_source.C_out, cold_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,-120},{-250,-120}}, color={28,108,200}));
  connect(cold_source_temperature_sensor.T_sensor, cold_source_temperature)
    annotation (Line(points={{-200,-110},{-200,-90}}, color={0,0,127}));
  connect(cold_source_pressure_sensor.C_out, cold_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,-120},{-210,-120}}, color={28,108,200}));
  connect(cold_source_flow_sensor.Q_sensor, cold_source_flow)
    annotation (Line(points={{-160,-110},{-160,-90}}, color={0,0,127}));
  connect(cold_source_temperature_sensor.C_out, cold_source_flow_sensor.C_in)
    annotation (Line(points={{-190,-120},{-170,-120}}, color={28,108,200}));
  connect(cold_source_humidity_sensor.H_sensor, cold_source_humidity)
    annotation (Line(points={{-120,-110},{-120,-90}}, color={0,0,127}));
  connect(cold_source_flow_sensor.C_out, cold_source_humidity_sensor.C_in)
    annotation (Line(points={{-150,-120},{-130,-120}}, color={28,108,200}));
  connect(cold_source_humidity_sensor.C_out, airCooledCondenser_with_subcooling.C_cold_in)
    annotation (Line(points={{-110,-120},{-10,-120},{-10,0}}, color={28,108,200}));
  connect(hot_source_flow_sensor.Q_sensor, hot_source_flow)
    annotation (Line(points={{-240,170},{-240,190}}, color={0,0,127}));
  connect(hot_source.C_out, hot_source_flow_sensor.C_in)
    annotation (Line(points={{-275,160},{-250,160}}, color={28,108,200}));
  connect(condensing_pressure_sensor.P_sensor, condensing_pressure)
    annotation (Line(points={{-80,100},{-80,120}}, color={0,0,127}));
  connect(hot_source_flow_sensor.C_out, condensing_pressure_sensor.C_in)
    annotation (Line(points={{-230,160},{-90,160},{-90,90}}, color={28,108,200}));
  connect(condensing_pressure_sensor.C_out, airCooledCondenser_with_subcooling.C_hot_in)
    annotation (Line(points={{-70,90},{0,90},{0,10}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.T_sensor, cold_sink_temperature)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(airCooledCondenser_with_subcooling.C_cold_out, cold_sink_temperature_sensor.C_in)
    annotation (Line(points={{10,0},{70,0},{70,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{90,-120},{165,-120}}, color={28,108,200}));
  connect(hot_sink_temperature_sensor.T_sensor, hot_sink_temperature)
    annotation (Line(points={{80,170},{80,190}}, color={0,0,127}));
  connect(airCooledCondenser_with_subcooling.C_hot_out, hot_sink_temperature_sensor.C_in)
    annotation (Line(points={{0,-10},{70,-10},{70,160}},
                                                       color={28,108,200}));
  connect(hot_sink_temperature_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{90,160},{165,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. Targeted test correction; see accompanying validation report.</p><p>Scalar API (no graphical component port): Kth_cond; Scalar API (no graphical component port): Kth_subc</p></html>"));
end AirCooledCondenser_with_subcooling_direct;
