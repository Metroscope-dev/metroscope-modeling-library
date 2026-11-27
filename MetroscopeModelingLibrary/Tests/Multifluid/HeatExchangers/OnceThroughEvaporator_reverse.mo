within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OnceThroughEvaporator_reverse "Isolated reverse test of a once-through HRSG evaporator"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.OnceThroughEvaporator evaporator(S=15000)
    annotation (Placement(transformation(extent={{-16,-56},{50,130}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={250,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_pressure_sensor(
    sensor_function="BC",
    causality="",
    P_start=130,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={210,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_temp_sensor(
    sensor_function="BC",
    causality="",
    T_start=305,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={170,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor cold_flow_sensor(
    sensor_function="BC",
    causality="",
    Q_start=85,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={130,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_out_pressure_sensor(
    sensor_function="Calibration",
    causality="evaporator_Kfr_cold",
    P_start=129.5,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-115,180}, rotation=180)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-160,180}, rotation=180)));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-250,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor hot_pressure_sensor(
    sensor_function="BC",
    causality="",
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-210,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_temp_sensor(
    sensor_function="BC",
    causality="",
    T_start=540,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-170,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor hot_flow_sensor(
    sensor_function="BC",
    causality="",
    Q_start=640,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-130,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_out_temp_sensor(
    T_start=398,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={115,-110})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={160,-110})));

  parameter Units.MassFraction hotComposition[FlueGasesMedium.nXi] =
    {0.752,0.135,0.052,0.061,0};

  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={210,150}),
      iconTransformation(extent={{-140,150},{-100,190}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={170,150}),
      iconTransformation(extent={{-140,95},{-100,135}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={130,150}),
      iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_out_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-115,150}),
      iconTransformation(extent={{-140,205},{-100,245}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-210,-80}),
      iconTransformation(extent={{-140,-15},{-100,25}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-170,-80}),
      iconTransformation(extent={{-140,-70},{-100,-30}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-130,-80}),
      iconTransformation(extent={{-140,-125},{-100,-85}})));
  Utilities.Interfaces.Observable                                 hot_out_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={115,-80}),
      iconTransformation(extent={{-140,-180},{-100,-140}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal evaporator_Kfr_cold(start=5000)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-81,110})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal evaporator_Kth(
    unit="W/(m2.K)", start=70)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-42,-72}, rotation=90)));
  Utilities.Interfaces.CalibrationInput cold_temp_out annotation (Placement(
        transformation(extent={{-4,-4},{4,4}}, origin={-88,150}),
        iconTransformation(extent={{-140,95},{-100,135}})));
  SensorsControl.WaterSteam.TemperatureSensor cold_temp_out_sensor(
    sensor_function="Calibration",
    causality="evaporator_Kth",
    T_start=340,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-88,180},
        rotation=180)));
equation
  hot_source.Xi_out = hotComposition;

  connect(cold_source.C_out, cold_pressure_sensor.C_in)
    annotation (Line(points={{245,180},{220,180}}, color={28,108,200}));
  connect(cold_pressure_sensor.C_out, cold_temp_sensor.C_in)
    annotation (Line(points={{200,180},{180,180}}, color={28,108,200}));
  connect(cold_temp_sensor.C_out, cold_flow_sensor.C_in)
    annotation (Line(points={{160,180},{140,180}}, color={28,108,200}));
  connect(cold_flow_sensor.C_out, evaporator.C_cold_in)
    annotation (Line(points={{120,180},{76,180},{76,44},{66,44},{66,-68},{17,
          -68},{17,-54.14}},                                      color={28,108,200}));
  connect(cold_out_pressure_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{-125,180},{-155,180}}, color={28,108,200}));
  connect(hot_source.C_out, hot_pressure_sensor.C_in)
    annotation (Line(points={{-245,-110},{-220,-110}}, color={95,95,95}));
  connect(hot_pressure_sensor.C_out, hot_temp_sensor.C_in)
    annotation (Line(points={{-200,-110},{-180,-110}}, color={95,95,95}));
  connect(hot_temp_sensor.C_out, hot_flow_sensor.C_in)
    annotation (Line(points={{-160,-110},{-140,-110}}, color={95,95,95}));
  connect(hot_flow_sensor.C_out, evaporator.C_hot_in)
    annotation (Line(points={{-120,-110},{-52,-110},{-52,37},{-14.35,37}},
                                                                      color={95,95,95}));
  connect(evaporator.C_hot_out, hot_out_temp_sensor.C_in)
    annotation (Line(points={{50,37},{100,37},{100,-110},{105,-110}},
                                                                  color={95,95,95}));
  connect(hot_out_temp_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{125,-110},{155,-110}}, color={95,95,95}));

  connect(cold_pressure_sensor.P_sensor, cold_pressure)
    annotation (Line(points={{210,170},{210,150}}, color={0,0,127}));
  connect(cold_temp_sensor.T_sensor, cold_temp)
    annotation (Line(points={{170,170},{170,150}}, color={0,0,127}));
  connect(cold_flow_sensor.Q_sensor, cold_flow)
    annotation (Line(points={{130,170},{130,150}}, color={0,0,127}));
  connect(cold_out_pressure_sensor.P_sensor, cold_out_pressure)
    annotation (Line(points={{-115,170},{-115,150}}, color={0,0,127}));
  connect(hot_pressure_sensor.P_sensor, hot_pressure)
    annotation (Line(points={{-210,-100},{-210,-80}}, color={0,0,127}));
  connect(hot_temp_sensor.T_sensor, hot_temp)
    annotation (Line(points={{-170,-100},{-170,-80}}, color={0,0,127}));
  connect(hot_flow_sensor.Q_sensor, hot_flow)
    annotation (Line(points={{-130,-100},{-130,-80}}, color={0,0,127}));
  connect(hot_out_temp_sensor.T_sensor, hot_out_temp)
    annotation (Line(points={{115,-100},{115,-80}}, color={0,0,127}));
  connect(evaporator_Kfr_cold, evaporator.Kfr_cold)
    annotation (Line(points={{-81,110},{-28,110},{-28,111.4},{-19.3,111.4}},
                                               color={0,0,127}));
  connect(evaporator_Kth, evaporator.Kth)
    annotation (Line(points={{-42,-72},{-42,-37.4},{-19.3,-37.4}},
                                                   color={0,0,127}));
  connect(cold_temp_out, cold_temp_out_sensor.T_sensor)
    annotation (Line(points={{-88,150},{-88,170}}, color={28,108,200}));
  connect(evaporator.C_cold_out, cold_temp_out_sensor.C_in) annotation (Line(
        points={{17,130},{17,180},{-78,180}}, color={28,108,200}));
  connect(cold_temp_out_sensor.C_out, cold_out_pressure_sensor.C_in)
    annotation (Line(points={{-98,180},{-105,180}}, color={28,108,200}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-280,-155},{280,215}})),
    Documentation(info="<html><p>Illustrative counterflow HRSG point. The exchange area and wet flue-gas composition are illustrative assumptions. The water enters subcooled and leaves in the two-phase range. The illustrative water pressures are 130 and 129.5 bar absolute. Water outlet pressure identifies evaporator_Kfr_cold, and gas outlet temperature identifies evaporator_Kth.</p><p>MML4 source interface inspected at commit 10344cb, Modelica Standard Library 4.1.0.</p></html>"));
end OnceThroughEvaporator_reverse;
