within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model TurbineLPCondenser_reverse_step2
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions

  input Units.SpecificEnthalpy source_h(start=2.9e6);

  // Hypothesis on component parameters
  // Turbines
  parameter Units.Area LP_turbine_area_nz = 12; // pi*turbine_R**2 - pi*rotor_R**2, with turbine_R = 2m, rotor_R = 0.35m

  // Extraction splitters

  parameter Units.Fraction LP_turbine3_ext_alpha = 1;

  // Condenser
  parameter Units.Area S = 100;
  parameter Units.Height water_height = 1;

  parameter Units.Pressure P_offset = 0;

  // Generator
  parameter Units.Yield generator_eta = 0.99;

  // Calibrated parameters
  // Components
  // Boundary conditions
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-180,-10},{-160,10}})));
  SensorsControl.WaterSteam.PressureSensor source_P_sensor(sensor_function="BC",
    causality="",
    P_start=15.5,
    signal_unit="barA",
    display_unit="barA",
    P_0=1550000,
    Q_0=1150,
    h_0=2900000) annotation (Placement(transformation(extent={{-160,-10},{-140,10}})));
  SensorsControl.WaterSteam.FlowSensor source_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=1150,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=1550000,
    Q_0=1150,
    h_0=2900000) annotation (Placement(transformation(extent={{-134,-10},{-114,10}})));

  // Turbines
  WaterSteam.Machines.SteamTurbine LP_turbine1(P_in_0=1500000,
    P_out_0=600000,
    h_in_0=2.9e6,
    Q_0=1150) annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
  WaterSteam.Machines.SteamTurbine LP_turbine2(P_in_0=600000,
    P_out_0=150000,
    Q_0=1150) annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  WaterSteam.Machines.SteamTurbineWithNozzle LP_turbine3(P_in_0=150000,
    P_out_0=5080,
    area_nz=LP_turbine_area_nz) annotation (Placement(transformation(extent={{38,-10},{58,10}})));

  // Extractions
  WaterSteam.Pipes.SteamExtractionSplitter LP_turbine1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter LP_turbine2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
  SensorsControl.WaterSteam.FlowSensor LP_turbine1_ext_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=78,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=560000,
    Q_0=78,
    h_0=2718409.25) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-50})));
  SensorsControl.WaterSteam.FlowSensor LP_turbine2_ext_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=75,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=175000,
    Q_0=75,
    h_0=2545215.5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-50})));
  SensorsControl.WaterSteam.PressureSensor LP_turbine1_ext_P_sensor(sensor_function="Calibration",
    causality="LP_turbine1_Cst",
    P_start=5.6,
    signal_unit="barA",
    display_unit="barA",
    P_0=560000,
    Q_0=78,
    h_0=2718409.25) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-25})));
  SensorsControl.WaterSteam.PressureSensor condenser_Psat_sensor(sensor_function="Calibration",
    causality="LP_turbine3_Cst",
    P_start=0.0508,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(0.0508),
    Q_0=997,
    h_0=2170909) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,0})));
  SensorsControl.WaterSteam.PressureSensor LP_turbine2_ext_P_sensor(sensor_function="Calibration",
    causality="LP_turbine2_Cst",
    P_start=1.75,
    signal_unit="barA",
    display_unit="barA",
    P_0=175000,
    Q_0=75,
    h_0=2545215.5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-25})));
  WaterSteam.BoundaryConditions.Sink LP_turbine1_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-76})));
  WaterSteam.BoundaryConditions.Sink LP_turbine2_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-78})));

  // Electricity
  Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{122,30},{142,50}})));
  Power.Machines.Generator generator(eta=generator_eta) annotation (Placement(transformation(extent={{68,28},{108,52}})));
  SensorsControl.Power.PowerSensor W_tot_sensor(sensor_function="Calibration",
    causality="LP_turbine3_eta_nz",
    W_start=760,
    signal_unit="MW",
    display_unit="MW") annotation (Placement(transformation(extent={{108,34},{120,46}})));
  // Condenser
  WaterSteam.HeatExchangers.Condenser condenser(faulty=false,
    Psat_0=5000,
    P_cold_in_0=500000,
    S=S,
    water_height=water_height,
    P_offset=P_offset) annotation (Placement(transformation(extent={{114,-60.6667},{146,-34}})));
  WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,-42})));
  WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{170,-60},{190,-40}})));
  SensorsControl.WaterSteam.TemperatureSensor cooling_source_T_sensor(sensor_function="BC",
    causality="",
    T_start=15,
    signal_unit="degC",
    display_unit="degC",
    T_0=273.15 + (15),
    P_0=500000,
    Q_0=22281.8378906,
    h_0=63375) annotation (Placement(transformation(extent={{60,-52},{80,-32}})));
  SensorsControl.WaterSteam.PressureSensor cooling_source_P_sensor(sensor_function="BC",
    causality="",
    P_start=5,
    signal_unit="barA",
    display_unit="barA",
    P_0=500000,
    Q_0=22281.8378906,
    h_0=63375) annotation (Placement(transformation(extent={{86,-52},{106,-32}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={130,-102})));
  SensorsControl.WaterSteam.TemperatureSensor cooling_sink_T_sensor(sensor_function="Calibration",
    causality="condenser_Kth",
    T_start=25,
    signal_unit="degC",
    display_unit="degC",
    T_0=273.15 + (25),
    P_0=3165.50488281,
    Q_0=22281.8378906,
    h_0=154295.046875) annotation (Placement(transformation(extent={{150,-60},{170,-40}})));
  // MML4 instrument interfaces and component characteristics.
  Utilities.Interfaces.BoundaryCondition source_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-150,30})));
  Utilities.Interfaces.BoundaryCondition source_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-124,30})));
  Utilities.Interfaces.BoundaryCondition LP_turbine1_ext_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-36,-50})));
  Utilities.Interfaces.BoundaryCondition LP_turbine2_ext_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={38,-50})));
  Utilities.Interfaces.CalibrationInput LP_turbine1_ext_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-36,-25})));
  Utilities.Interfaces.CalibrationInput LP_turbine2_ext_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={38,-25})));
  Utilities.Interfaces.CalibrationInput W_tot annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={114,66})));
  Utilities.Interfaces.BoundaryCondition P_cold_source annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={96,-12})));
  Utilities.Interfaces.BoundaryCondition T_cold_source annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={70,-12})));
  Utilities.Interfaces.CalibrationInput condenser_Psat annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={90,30})));
  Utilities.Interfaces.CalibrationInput cooling_sink_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={160,-20})));
  Utilities.Interfaces.RealOutput LP_turbine1_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-108,-44})));
  Utilities.Interfaces.RealOutput LP_turbines_eta_is annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-115,-110})));
  Utilities.Interfaces.RealOutput LP_turbine1_ext_alpha annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-46,24})));
  Utilities.Interfaces.RealOutput LP_turbine2_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-42,-44})));
  Utilities.Interfaces.RealOutput LP_turbine2_ext_alpha annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={28,24})));
  Utilities.Interfaces.RealOutput LP_turbine3_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={36,24})));
  Utilities.Interfaces.RealOutput LP_turbine3_eta_nz annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={70,-30})));
  Utilities.Interfaces.RealOutput C_incond annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={110,-82})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={90,-76})));
  Utilities.Interfaces.RealOutput condenser_Kth annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={90,-24})));

equation
  // Boundary conditions

  source.h_out = source_h;

  // Condenser


  connect(LP_turbine1.C_out, LP_turbine1_ext.C_in) annotation (Line(points={{-86,0},{-76.6,0},{-76.6,-1}},  color={28,108,200}));
  connect(LP_turbine1_ext.C_main_out, LP_turbine2.C_in) annotation (Line(points={{-55.4,-1},{-40,-1},{-40,0}}, color={28,108,200}));
  connect(LP_turbine2.C_out, LP_turbine2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0},{-2.6,-1}},  color={28,108,200}));
  connect(LP_turbine2_ext.C_main_out, LP_turbine3.C_in) annotation (Line(points={{18.6,-1},{38,-1},{38,0}}, color={28,108,200}));
  connect(LP_turbine3.C_W_out, generator.C_in) annotation (Line(points={{54,8},{68,8},{68,40},{75.6,40}},      color={244,125,35}));
  connect(LP_turbine2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{75.6,40}},  color={244,125,35}));
  connect(LP_turbine1.C_W_out, generator.C_in) annotation (Line(points={{-86,8.4},{-74,8.4},{-74,40},{75.6,40}},   color={244,125,35}));
  connect(LP_turbine1_ext.C_ext_out, LP_turbine1_ext_P_sensor.C_in) annotation (Line(points={{-66,-7.12},{-66,-15}}, color={28,108,200}));
  connect(LP_turbine2_ext.C_ext_out, LP_turbine2_ext_P_sensor.C_in) annotation (Line(points={{8,-7.12},{8,-15}}, color={28,108,200}));
  connect(powerSink.C_in, W_tot_sensor.C_out) annotation (Line(points={{127,40},{119.88,40}}, color={244,125,35}));
  connect(W_tot_sensor.C_in, generator.C_out) annotation (Line(points={{108,40},{102,40}}, color={244,125,35}));
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-160,0},{-165,0}}, color={28,108,200}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in) annotation (Line(points={{-140,0},{-134,0}}, color={28,108,200}));
  connect(LP_turbine1.C_in, source_Q_sensor.C_out) annotation (Line(points={{-106,0},{-114,0}}, color={28,108,200}));
  connect(LP_turbine1_ext_P_sensor.C_out, LP_turbine1_ext_Q_sensor.C_in) annotation (Line(points={{-66,-35},{-66,-40}}, color={28,108,200}));
  connect(LP_turbine1_ext_Q_sensor.C_out, LP_turbine1_ext_sink.C_in) annotation (Line(points={{-66,-60},{-66,-71}}, color={28,108,200}));
  connect(LP_turbine2_ext_P_sensor.C_out, LP_turbine2_ext_Q_sensor.C_in) annotation (Line(points={{8,-35},{8,-40}}, color={28,108,200}));
  connect(LP_turbine2_ext_Q_sensor.C_out, LP_turbine2_ext_sink.C_in) annotation (Line(points={{8,-60},{8,-73}}, color={28,108,200}));
  connect(LP_turbine3.C_out, condenser_Psat_sensor.C_in) annotation (Line(points={{58,0},{80,0}}, color={28,108,200}));
  connect(condenser.C_hot_in, condenser_Psat_sensor.C_out) annotation (Line(points={{130,-34},{130,0},{100,0}}, color={28,108,200}));
  connect(cooling_source.C_out, cooling_source_T_sensor.C_in) annotation (Line(points={{55,-42},{60,-42}}, color={28,108,200}));
  connect(cooling_source_T_sensor.C_out, cooling_source_P_sensor.C_in) annotation (Line(points={{80,-42},{86,-42}}, color={28,108,200}));
  connect(condenser.C_cold_in, cooling_source_P_sensor.C_out) annotation (Line(points={{146,-44.66668},{146,-20},{106,-20},{106,-42}},    color={28,108,200}));
  connect(sink.C_in, condenser.C_hot_out) annotation (Line(points={{130,-97},{130,-60.6667}}, color={28,108,200}));
  connect(condenser.C_cold_out, cooling_sink_T_sensor.C_in) annotation (Line(points={{146,-52.66669},{149,-52.66669},{149,-48.8148},{149,-50},{150,-50}},
                                                                                                                                         color={28,108,200}));
  connect(cooling_sink.C_in, cooling_sink_T_sensor.C_out) annotation (Line(points={{175,-50},{170,-50}}, color={28,108,200}));

  // MML4 signal connections and fixed characteristics.
  connect(source_P_sensor.P_sensor, source_P) annotation (Line(points={{-150,10},{-150,30}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q) annotation (Line(points={{-124,10},{-124,30}}, color={0,0,127}));
  connect(LP_turbine1_ext_Q_sensor.Q_sensor, LP_turbine1_ext_Q) annotation (Line(points={{-56,-50},{-36,-50}}, color={0,0,127}));
  connect(LP_turbine2_ext_Q_sensor.Q_sensor, LP_turbine2_ext_Q) annotation (Line(points={{18,-50},{38,-50}}, color={0,0,127}));
  connect(LP_turbine1_ext_P_sensor.P_sensor, LP_turbine1_ext_P) annotation (Line(points={{-56,-25},{-36,-25}}, color={0,0,127}));
  connect(LP_turbine2_ext_P_sensor.P_sensor, LP_turbine2_ext_P) annotation (Line(points={{18,-25},{38,-25}}, color={0,0,127}));
  connect(W_tot_sensor.W_sensor, W_tot) annotation (Line(points={{114,46},{114,66}}, color={0,0,127}));
  connect(cooling_source_P_sensor.P_sensor, P_cold_source) annotation (Line(points={{96,-32},{96,-12}}, color={0,0,127}));
  connect(cooling_source_T_sensor.T_sensor, T_cold_source) annotation (Line(points={{70,-32},{70,-12}}, color={0,0,127}));
  connect(condenser_Psat_sensor.P_sensor, condenser_Psat) annotation (Line(points={{90,10},{90,30}}, color={0,0,127}));
  connect(cooling_sink_T_sensor.T_sensor, cooling_sink_T) annotation (Line(points={{160,-40},{160,-20}}, color={0,0,127}));
  connect(LP_turbine1_Cst, LP_turbine1.Cst) annotation (Line(points={{-108,-44},{-102,-44},{-102,-21},{-102,-7}}, color={0,0,127}));
  connect(LP_turbines_eta_is, LP_turbine1.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{-98,-106},{-98,-8}}, color={0,0,127}));
  connect(LP_turbines_eta_is, LP_turbine2.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{-32,-106},{-32,-8}}, color={0,0,127}));
  connect(LP_turbines_eta_is, LP_turbine3.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{47.3,-106},{47.3,8.7}}, color={0,0,127}));
  LP_turbines_eta_is = 0.9;
  connect(LP_turbine1_ext_alpha, LP_turbine1_ext.alpha) annotation (Line(points={{-46,24},{-66,24},{-66,-9.6},{-58,-9.6},{-58,1.16}}, color={0,0,127}));
  LP_turbine1_ext_alpha = 1;
  connect(LP_turbine2_Cst, LP_turbine2.Cst) annotation (Line(points={{-42,-44},{-36,-44},{-36,-21},{-36,-7}}, color={0,0,127}));
  connect(LP_turbine2_ext_alpha, LP_turbine2_ext.alpha) annotation (Line(points={{28,24},{8,24},{8,-9.6},{16,-9.6},{16,1.16}}, color={0,0,127}));
  LP_turbine2_ext_alpha = 1;
  connect(LP_turbine3_Cst, LP_turbine3.Cst) annotation (Line(points={{36,24},{41.4,24},{41.4,-6.4},{41.4,7.6}}, color={0,0,127}));
  connect(LP_turbine3_eta_nz, LP_turbine3.eta_nz) annotation (Line(points={{70,-30},{55.7,-30},{55.7,-20.9},{55.7,-6.9}}, color={0,0,127}));
  connect(C_incond, condenser.C_incond) annotation (Line(points={{110,-82},{113.36,-82},{113.36,-56},{113.36,-42.00001}}, color={0,0,127}));
  C_incond = 0;
  connect(Kfr_cold, condenser.Kfr_cold) annotation (Line(points={{90,-76},{113.36,-76},{113.36,-64},{113.36,-50.00002}}, color={0,0,127}));
  Kfr_cold = 1;
  connect(condenser_Kth, condenser.Kth) annotation (Line(points={{90,-24},{113.36,-24},{113.36,-58.6667},{113.36,-44.66668}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-180,-140},{200,140}})), Icon(coordinateSystem(extent={{-100,-100},{100,100}}), graphics={
                               Polygon(
          points={{-100,60},{-100,40},{-100,-40},{-100,-60},{-80,-66},{80,-100},{100,-100},{100,-80},{100,77.539},{100,100},{80,100},{-80,68},{-100,60}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{-92,58},{-92,40},{-92,-40},{-92,-54},{-74,-60},{72,-90},{92,-94},{92,-72},{92,70},{92,92},{72,90},{-72,62},{-92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{66,86},{66,-86}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{22,78},{22,-78}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-20,68},{-20,-68}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-60,60},{-60,-58}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{74,2},{-76,-2}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end TurbineLPCondenser_reverse_step2;
