within MetroscopeModelingLibrary;
model CCGT_1P_MML4_direct_initialized
  "MML4 direct CCGT - organized diagram, pressure cut and loop breaker before feedwater pump"
  // MML4 top-level boundary-condition / observable signals are graphical connectors.
  // Initialization P_0 in Pa, T_0 in K, Q_0 in kg/s; displayed P_start in barA and T_start in degC.
  // Sensor units: degC, barA and kg/s. Electrical power is reported internally in MW.

  FlueGases.BoundaryConditions.Source sourceAir(P_0=101300, Q_0=500, h_0=3e5) annotation (Placement(transformation(extent={{-240,-10},{-220,10}})));
  FlueGases.Pipes.Filter airFilter annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  FlueGases.Machines.AirCompressor airCompressor(h_out(start=7e5)) annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  FlueGases.Machines.GasTurbine gasTurbine annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  FlueGases.BoundaryConditions.Sink exhaustSink annotation (Placement(transformation(extent={{660,-10},{680,10}})));
  Fuel.BoundaryConditions.Source sourceFuel(P_0=3e6, Q_0=15, h_0=9e5) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-100})));
  Power.Machines.Generator generator annotation (Placement(transformation(extent={{130,70},{150,90}})));
  Power.BoundaryConditions.Sink grid annotation (Placement(transformation(extent={{210,70},{230,90}})));

  SensorsControl.FlueGases.TemperatureSensor ambientTemperatureSensor(sensor_function="BC", T_start=15, signal_unit="degC", display_unit="degC", T_0=288.15) annotation (Placement(transformation(extent={{-212,-10},{-192,10}})));
  SensorsControl.FlueGases.PressureSensor ambientPressureSensor(sensor_function="BC", P_start=1.013, signal_unit="barA", display_unit="barA", P_0=101300) annotation (Placement(transformation(extent={{-190,-10},{-170,10}})));
  SensorsControl.FlueGases.FlowSensor airFlowSensor(sensor_function="BC", Q_start=500, signal_unit="kg/s", display_unit="kg/s", Q_0=500) annotation (Placement(transformation(extent={{-168,-10},{-148,10}})));
  SensorsControl.FlueGases.PressureSensor filterOutletPressureSensor(sensor_function="Calibration", causality="airFilter_Kfr", P_start=1.003, signal_unit="barA", display_unit="barA", P_0=100300) annotation (Placement(transformation(extent={{-128,-10},{-108,10}})));
  SensorsControl.FlueGases.PressureSensor compressorOutletPressureSensor(sensor_function="Calibration", causality="airCompressor_tau", P_start=15, signal_unit="barA", display_unit="barA", P_0=1500000) annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  SensorsControl.FlueGases.TemperatureSensor compressorOutletTemperatureSensor(sensor_function="Calibration", causality="airCompressor_eta_is", T_start=400, signal_unit="degC", display_unit="degC", T_0=673.15) annotation (Placement(transformation(extent={{-36,-10},{-16,10}})));
  SensorsControl.FlueGases.TemperatureSensor turbineInletTemperatureSensor(sensor_function="Calibration", causality="combustionChamber_eta", T_start=1300, signal_unit="degC", display_unit="degC", T_0=1573.15) annotation (Placement(transformation(extent={{16,-10},{36,10}})));
  SensorsControl.FlueGases.PressureSensor turbineOutletPressureSensor(sensor_function="BC", P_start=1.03, signal_unit="barA", display_unit="barA", P_0=103000) annotation (Placement(transformation(extent={{98,-10},{118,10}})));
  SensorsControl.FlueGases.TemperatureSensor turbineOutletTemperatureSensor(sensor_function="Calibration", causality="gasTurbine_eta_is", T_start=600, signal_unit="degC", display_unit="degC", T_0=873.15) annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  SensorsControl.Fuel.PressureSensor fuelPressureSensor(sensor_function="BC", P_start=30, signal_unit="barA", display_unit="barA", P_0=3000000) annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
  SensorsControl.Fuel.FlowSensor fuelFlowSensor(sensor_function="BC", Q_start=15, signal_unit="kg/s", display_unit="kg/s", Q_0=15) annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  SensorsControl.Power.PowerSensor electricalPowerSensor(sensor_function="Unidentified", W_start=200) annotation (Placement(transformation(extent={{168,70},{188,90}})));

  Utilities.Interfaces.BoundaryCondition ambientTemperature annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-202,25})));
  Utilities.Interfaces.BoundaryCondition ambientPressure annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-180,25})));
  Utilities.Interfaces.BoundaryCondition airFlow annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-158,25})));
  Utilities.Interfaces.BoundaryCondition fuelPressure annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={28,-80})));
  Utilities.Interfaces.BoundaryCondition fuelFlow annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={28,-50})));
  Utilities.Interfaces.BoundaryCondition turbineOutletPressure annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={108,25})));

  Utilities.Interfaces.Observable filterOutletPressure annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-118,25})));
  Utilities.Interfaces.Observable compressorOutletPressure annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-48,25})));
  Utilities.Interfaces.Observable compressorOutletTemperature annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-26,25})));
  Utilities.Interfaces.Observable turbineInletTemperature annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={26,25})));
  Utilities.Interfaces.Observable turbineOutletTemperature annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={130,25})));
  Utilities.Interfaces.Observable electricalPower annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={178,105})));

  Utilities.Interfaces.GenericReal airFilter_Kfr annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-140,22})));
  Utilities.Interfaces.GenericReal airCompressor_tau annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={-76,-24})));
  Utilities.Interfaces.GenericReal airCompressor_eta_is annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={-64,-24})));
  Utilities.Interfaces.GenericReal combustionChamber_Kfr annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={8,42})));
  Utilities.Interfaces.GenericReal combustionChamber_eta annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-8,42})));
  Utilities.Interfaces.GenericReal gasTurbine_eta_is annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={78,-26})));

  // Single-pressure heat recovery steam generator (flue gas path: SH -> EVAP -> ECO).
  MultiFluid.HeatExchangers.Superheater superheater(
    Q_cold_0=85,
    Q_hot_0=515,
    T_cold_in_0=273.15+325,
    T_cold_out_0=273.15+540,
    T_hot_in_0=273.15+600,
    T_hot_out_0=273.15+450,
    P_cold_in_0=122e5,
    P_cold_out_0=120e5,
    P_hot_in_0=1.06e5,
    P_hot_out_0=1.055e5,
    h_cold_in_0=2.72e6,
    h_cold_out_0=3.49e6,
    h_hot_in_0=1.05e6,
    h_hot_out_0=0.80e6)
    annotation (Placement(transformation(extent={{240,-45},{320,45}})));
  MultiFluid.HeatExchangers.Evaporator evaporator(
    Q_cold_0=85,
    Q_hot_0=515,
    T_cold_in_0=273.15+310,
    T_hot_in_0=273.15+450,
    T_hot_out_0=273.15+330,
    P_cold_in_0=125e5,
    P_cold_out_0=122e5,
    P_hot_in_0=1.055e5,
    P_hot_out_0=1.05e5,
    h_cold_in_0=1.42e6,
    h_hot_in_0=0.80e6,
    h_hot_out_0=0.62e6)
    annotation (Placement(transformation(extent={{350,-45},{450,45}})));
  MultiFluid.HeatExchangers.Economiser economiser(
    Q_cold_0=85,
    Q_hot_0=515,
    T_cold_in_0=273.15+120,
    T_cold_out_0=273.15+310,
    T_hot_in_0=273.15+330,
    T_hot_out_0=273.15+160,
    P_cold_in_0=128e5,
    P_cold_out_0=125e5,
    P_hot_in_0=1.05e5,
    P_hot_out_0=1.045e5,
    h_cold_in_0=0.51e6,
    h_cold_out_0=1.42e6,
    h_hot_in_0=0.62e6,
    h_hot_out_0=0.44e6)
    annotation (Placement(transformation(extent={{480,-45},{560,45}})));

  // Steam/water closed Rankine cycle.
  WaterSteam.Machines.SteamTurbine steamTurbine(
    Q_0=85,
    P_in_0=120e5,
    P_out_0=0.07e5,
    h_in_0=3.49e6,
    h_out_0=2.35e6,
    h_is(start=2.1e6))
    annotation (Placement(transformation(extent={{540,-190},{580,-150}})));
  WaterSteam.HeatExchangers.Condenser steamCondenser(
    Q_hot_0=85,
    Q_cold_0=5000,
    Psat_0=0.07e5,
    T_cold_in_0=273.15+15,
    T_cold_out_0=273.15+25,
    P_cold_in_0=5e5,
    P_cold_out_0=4.9e5,
    h_hot_in_0=2.35e6,
    h_cold_in_0=0.063e6,
    h_cold_out_0=0.105e6,
    P_tot(start=0.07e5),
    Psat(start=0.07e5,
    nominal=0.07e5))
    annotation (Placement(transformation(extent={{410,-340},{470,-280}})));
  WaterSteam.Machines.FixedSpeedPump feedwaterPump(
    Q_0=85,
    P_in_0=0.08e5,
    P_out_0=128e5,
    h_in_0=0.18e6,
    h_out_0=0.20e6)
    annotation (Placement(transformation(extent={{150,-340},{190,-300}})));

  // Hydraulic decoupling immediately upstream of the feedwater pump.
  WaterSteam.Pipes.PressureCut feedwaterPressureCut(
    Q_0=85,
    P_in_0=0.08e5,
    P_out_0=0.08e5,
    h_0=0.18e6)
    annotation (Placement(transformation(extent={{30,-340},{50,-320}})));
  WaterSteam.Pipes.LoopBreaker feedwaterLoopBreaker(loop_flow_error(start=0))
    annotation (Placement(transformation(extent={{85,-340},{105,-320}})));

  // Independent cooling-water loop on condenser cold side.
  WaterSteam.BoundaryConditions.Source coolingWaterSource(P_0=5e5, Q_0=5000, h_0=0.063e6) annotation (Placement(transformation(extent={{520,-380},{540,-360}})));
  WaterSteam.BoundaryConditions.Sink coolingWaterSink annotation (Placement(transformation(extent={{620,-380},{640,-360}})));
  SensorsControl.WaterSteam.TemperatureSensor coolingWaterInTSensor(sensor_function="BC", T_start=15, T_0=288.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{548,-380},{568,-360}})));
  SensorsControl.WaterSteam.PressureSensor coolingWaterInPSensor(sensor_function="BC", P_start=5, P_0=500000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{574,-380},{594,-360}})));
  SensorsControl.WaterSteam.FlowSensor coolingWaterInQSensor(sensor_function="BC", Q_start=5000, Q_0=5000, signal_unit="kg/s", display_unit="kg/s") annotation (Placement(transformation(extent={{596,-380},{616,-360}})));

  // Six steam-cycle component outlets: pressure and temperature measurements.
  SensorsControl.WaterSteam.PressureSensor pumpPSensor(sensor_function="Calibration", causality="feedwaterPump_hn", P_start=128, P_0=12800000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{150,-270},{170,-250}})));
  SensorsControl.WaterSteam.TemperatureSensor pumpTSensor(sensor_function="Calibration", causality="feedwaterPump_rh", T_start=120, T_0=393.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{174,-270},{194,-250}})));
  SensorsControl.WaterSteam.PressureSensor ecoPSensor(sensor_function="Calibration", causality="economiser_Kfr_cold", P_start=125, P_0=12500000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{488,-112},{508,-92}})));
  SensorsControl.WaterSteam.TemperatureSensor ecoTSensor(sensor_function="Calibration", causality="economiser_Kth", T_start=310, T_0=583.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{512,-112},{532,-92}})));
  SensorsControl.WaterSteam.PressureSensor evapPSensor(sensor_function="Calibration", causality="evaporator_Kth", P_start=122, P_0=12200000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{380,-112},{400,-92}})));
  SensorsControl.WaterSteam.TemperatureSensor evapTSensor(sensor_function="Calibration", causality="evaporator_Kth", T_start=325, T_0=598.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{404,-112},{424,-92}})));
  SensorsControl.WaterSteam.PressureSensor shPSensor(sensor_function="Calibration", causality="superheater_Kfr_cold", P_start=120, P_0=12000000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{260,-112},{280,-92}})));
  SensorsControl.WaterSteam.TemperatureSensor shTSensor(sensor_function="Calibration", causality="superheater_Kth", T_start=540, T_0=813.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{284,-112},{304,-92}})));
  SensorsControl.WaterSteam.PressureSensor stPSensor(sensor_function="Calibration", causality="steamTurbine_Cst", P_start=0.07, P_0=7000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{540,-245},{560,-225}})));
  SensorsControl.WaterSteam.TemperatureSensor stTSensor(sensor_function="Calibration", causality="steamTurbine_eta_is", T_start=45, T_0=318.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{564,-245},{584,-225}})));
  SensorsControl.WaterSteam.PressureSensor condPSensor(sensor_function="Calibration", causality="steamCondenser_Kth", P_start=0.08, P_0=8000, signal_unit="barA", display_unit="barA") annotation (Placement(transformation(extent={{420,-405},{440,-385}})));
  SensorsControl.WaterSteam.TemperatureSensor condTSensor(sensor_function="Calibration", causality="steamCondenser_Qv_cold_in", T_start=42, T_0=315.15, signal_unit="degC", display_unit="degC") annotation (Placement(transformation(extent={{444,-405},{464,-385}})));
  SensorsControl.WaterSteam.FlowSensor steamTurbineInQSensor(sensor_function="Unidentified", Q_start=85, Q_0=85, signal_unit="kg/s", display_unit="kg/s") annotation (Placement(transformation(extent={{500,-190},{520,-170}})));
  SensorsControl.WaterSteam.FlowSensor feedwaterPumpOutQSensor(sensor_function="Unidentified", Q_start=85, Q_0=85, signal_unit="kg/s", display_unit="kg/s") annotation (Placement(transformation(extent={{200,-340},{220,-320}})));

  // Graphic interfaces: BC inputs, computed observables, and component parameters.
  Utilities.Interfaces.Observable pumpP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={170,-230})));
  Utilities.Interfaces.Observable pumpT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={194,-230})));
  Utilities.Interfaces.Observable ecoP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={498,-87})));
  Utilities.Interfaces.Observable ecoT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={522,-87})));
  Utilities.Interfaces.Observable evapP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={390,-87})));
  Utilities.Interfaces.Observable evapT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={414,-87})));
  Utilities.Interfaces.Observable shP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={270,-87})));
  Utilities.Interfaces.Observable shT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={294,-87})));
  Utilities.Interfaces.Observable stP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={560,-205})));
  Utilities.Interfaces.Observable stT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={584,-205})));
  Utilities.Interfaces.Observable condP annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={440,-365})));
  Utilities.Interfaces.Observable condT annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={464,-365})));
  Utilities.Interfaces.Observable steamTurbineInQ annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=270,origin={510,-135})));
  Utilities.Interfaces.Observable feedwaterPumpOutQ annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=270,origin={210,-285})));
  Utilities.Interfaces.BoundaryCondition coolingWaterInT annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=270,origin={558,-340})));
  Utilities.Interfaces.BoundaryCondition coolingWaterInP annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=270,origin={584,-340})));
  Utilities.Interfaces.BoundaryCondition coolingWaterInQ annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=270,origin={606,-340})));
  Utilities.Interfaces.GenericReal economiser_Kth annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={480,-68})));
  Utilities.Interfaces.GenericReal economiser_Kfr_cold annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={550,-68})));
  Utilities.Interfaces.GenericReal evaporator_Kth annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={365,-66})));
  Utilities.Interfaces.GenericReal superheater_Kth annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={235,-66})));
  Utilities.Interfaces.GenericReal superheater_Kfr_cold annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={321,-66})));
  Utilities.Interfaces.GenericReal steamTurbine_Cst annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={548,-125})));
  Utilities.Interfaces.GenericReal steamTurbine_eta_is annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={592,-125})));
  Utilities.Interfaces.GenericReal feedwaterPump_hn annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={158,-365})));
  Utilities.Interfaces.GenericReal feedwaterPump_rh annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={194,-365})));
  Utilities.Interfaces.GenericReal steamCondenser_Kth annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={388,-345})));
  Utilities.Interfaces.GenericReal steamCondenser_Kfr_cold annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={393,-375})));
  Utilities.Interfaces.GenericReal steamCondenser_Qv_cold_in annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={485,-345})));
  Utilities.Interfaces.GenericReal steamCondenser_C_incond annotation (Placement(transformation(extent={{-4,-4},{4,4}},origin={485,-375})));

  inner parameter Boolean show_causality = true;
  inner parameter Boolean display_output = true;

  // HX/steam-cycle component initialization is also explicit (steady-state guesses only).
  // Sensor initialization is explicit: T_start [degC], T_0 [K],
  // P_start [barA], P_0 [Pa], and Q_start / Q_0 [kg/s].

equation
  // Direct causality: component characteristics imposed, observables calculated.
  airFilter_Kfr = 0.004; // Initial filter pressure loss approximately 0.01 bar at 500 kg/s
  airCompressor_tau = 16;
  airCompressor_eta_is = 0.90;
  combustionChamber_Kfr = 0.1;
  combustionChamber_eta = 0.999;
  gasTurbine_eta_is = 0.89;

  // Mixture composition (air and simplified natural gas).
  sourceAir.Xi_out = {0.768,0.232,0,0,0};
  sourceFuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};
  sourceFuel.h_out = 0.9e6;

  connect(sourceAir.C_out,ambientTemperatureSensor.C_in) annotation (Line(points={{-226,0},{-212,0}}, color={95,95,95}));
  connect(ambientTemperatureSensor.C_out,ambientPressureSensor.C_in) annotation (Line(points={{-192,0},{-190,0}}, color={95,95,95}));
  connect(ambientPressureSensor.C_out,airFlowSensor.C_in) annotation (Line(points={{-170,0},{-168,0}}, color={95,95,95}));
  connect(airFlowSensor.C_out,airFilter.C_in) annotation (Line(points={{-148,0},{-150,0}}, color={95,95,95}));
  connect(airFilter.C_out,filterOutletPressureSensor.C_in) annotation (Line(points={{-130,0},{-128,0}}, color={95,95,95}));
  connect(filterOutletPressureSensor.C_out,airCompressor.C_in) annotation (Line(points={{-108,0},{-80,0}}, color={95,95,95}));
  connect(airCompressor.C_out,compressorOutletPressureSensor.C_in) annotation (Line(points={{-60,0},{-58,0}}, color={95,95,95}));
  connect(compressorOutletPressureSensor.C_out,compressorOutletTemperatureSensor.C_in) annotation (Line(points={{-38,0},{-36,0}}, color={95,95,95}));
  connect(compressorOutletTemperatureSensor.C_out,combustionChamber.inlet) annotation (Line(points={{-16,0},{-10,0}}, color={95,95,95}));
  connect(combustionChamber.outlet,turbineInletTemperatureSensor.C_in) annotation (Line(points={{10,0},{16,0}}, color={95,95,95}));
  connect(turbineInletTemperatureSensor.C_out,gasTurbine.C_in) annotation (Line(points={{36,0},{70,0}}, color={95,95,95}));
  connect(gasTurbine.C_out,turbineOutletPressureSensor.C_in) annotation (Line(points={{90,0},{98,0}}, color={95,95,95}));
  connect(turbineOutletPressureSensor.C_out,turbineOutletTemperatureSensor.C_in) annotation (Line(points={{118,0},{120,0}}, color={95,95,95}));

  connect(sourceFuel.C_out,fuelPressureSensor.C_in) annotation (Line(points={{0,-95},
          {0,-88},{0,-80},{-10,-80}},                                                          color={213,213,0}));
  connect(fuelPressureSensor.C_out,fuelFlowSensor.C_in) annotation (Line(points={{10,-80},
          {0,-80},{0,-50},{-10,-50}},                                                              color={213,213,0}));
  connect(fuelFlowSensor.C_out,combustionChamber.inlet1) annotation (Line(points={{10,-50},
          {10,-30},{0,-30},{0,-10}},                                                                color={213,213,0}));

  // Shared shaft: turbine drives compressor, surplus shaft power goes to generator.
  connect(gasTurbine.C_W_shaft,airCompressor.C_W_in) annotation (Line(points={{90,10},
          {90,55},{-60,55},{-60,7.5}},                                                                           color={244,125,35}));
  connect(gasTurbine.C_W_shaft,generator.C_in) annotation (Line(points={{90,10},{110,10},{110,80},{134,80}}, color={244,125,35}));
  connect(generator.C_out,electricalPowerSensor.C_in) annotation (Line(points={{147,80},{168,80}}, color={244,125,35}));
  connect(electricalPowerSensor.C_out,grid.C_in) annotation (Line(points={{188,80},{215,80}}, color={244,125,35}));

  connect(ambientTemperatureSensor.T_sensor,ambientTemperature) annotation (Line(points={{-202,10},{-202,25}}, color={0,0,127}));
  connect(ambientPressureSensor.P_sensor,ambientPressure) annotation (Line(points={{-180,10},{-180,25}}, color={0,0,127}));
  connect(airFlowSensor.Q_sensor,airFlow) annotation (Line(points={{-158,10},{-158,25}}, color={0,0,127}));
  connect(fuelPressureSensor.P_sensor,fuelPressure) annotation (Line(points={{0,-70},{28,-70},{28,-80}}, color={0,0,127}));
  connect(fuelFlowSensor.Q_sensor,fuelFlow) annotation (Line(points={{0,-40},{28,-40},{28,-50}}, color={0,0,127}));
  connect(turbineOutletPressureSensor.P_sensor,turbineOutletPressure) annotation (Line(points={{108,10},{108,25}}, color={0,0,127}));
  connect(filterOutletPressureSensor.P_sensor,filterOutletPressure) annotation (Line(points={{-118,10},{-118,25}}, color={0,0,127}));
  connect(compressorOutletPressureSensor.P_sensor,compressorOutletPressure) annotation (Line(points={{-48,10},{-48,25}}, color={0,0,127}));
  connect(compressorOutletTemperatureSensor.T_sensor,compressorOutletTemperature) annotation (Line(points={{-26,10},{-26,25}}, color={0,0,127}));
  connect(turbineInletTemperatureSensor.T_sensor,turbineInletTemperature) annotation (Line(points={{26,10},{26,25}}, color={0,0,127}));
  connect(turbineOutletTemperatureSensor.T_sensor,turbineOutletTemperature) annotation (Line(points={{130,10},{130,25}}, color={0,0,127}));
  connect(electricalPowerSensor.W_sensor,electricalPower) annotation (Line(points={{178,90},{178,105}}, color={0,0,127}));

  connect(airFilter.Kfr,airFilter_Kfr) annotation (Line(points={{-140,4},{-140,
          22}},                                                                       color={0,0,127}));
  connect(airCompressor.tau,airCompressor_tau) annotation (Line(points={{-76,
          -7.5},{-76,-24}},                                                                   color={0,0,127}));
  connect(airCompressor.eta_is,airCompressor_eta_is) annotation (Line(points={{-70,
          -6.25},{-70,-24},{-64,-24}},                                                                        color={0,0,127}));
  connect(combustionChamber.Kfr,combustionChamber_Kfr) annotation (Line(points={{4,10},{8,10},{8,42}}, color={0,0,127}));
  connect(combustionChamber.eta,combustionChamber_eta) annotation (Line(points={{-4,10},{-8,10},{-8,42}}, color={0,0,127}));
  connect(gasTurbine.eta_is,gasTurbine_eta_is) annotation (Line(points={{78,-8},
          {78,-26}},                                                                        color={0,0,127}));

  // MML4 direct: parameter connectors are specified, observables are calculated.
  economiser_Kth = 1000; economiser_Kfr_cold = 0.03;
  evaporator_Kth = 1000;
  superheater_Kth = 1000; superheater_Kfr_cold = 0.03;
  steamTurbine_Cst = 2000; steamTurbine_eta_is = 0.85;
  feedwaterPump_hn = 1400; feedwaterPump_rh = 0.78;
  steamCondenser_Kth = 1200; steamCondenser_Kfr_cold = 0.05;

  // Cooling-water flow is prescribed only by coolingWaterInQSensor (BC).
  steamCondenser_C_incond = 0;

  connect(turbineOutletTemperatureSensor.C_out,superheater.C_hot_in) annotation (Line(points={{140,0},
          {264,0}},                                                                                             color={95,95,95}));
  connect(superheater.C_hot_out,evaporator.C_hot_in) annotation (Line(points={{296,0},
          {328,0},{328,-21.3158},{360,-21.3158}},                                               color={95,95,95}));
  connect(evaporator.C_hot_out,economiser.C_hot_in) annotation (Line(points={{440,
          -21.3158},{460,-21.3158},{460,0},{504,0}},                                           color={95,95,95}));
  connect(economiser.C_hot_out,exhaustSink.C_in) annotation (Line(points={{536,0},
          {665,0}},                                                                         color={95,95,95}));
  connect(feedwaterPump.C_out,feedwaterPumpOutQSensor.C_in) annotation (Line(points={{190,-320},{200,-330}}, color={28,108,200}));
  connect(feedwaterPumpOutQSensor.C_out,pumpPSensor.C_in) annotation (Line(points={{220,-330},{150,-260}}, color={28,108,200}));
  connect(pumpPSensor.C_out,pumpTSensor.C_in) annotation (Line(points={{170,-260},{174,-260}}, color={28,108,200}));
  connect(pumpTSensor.C_out,economiser.C_cold_in) annotation (Line(points={{194,-260},{520,-45}}, color={28,108,200}));
  connect(economiser.C_cold_out,ecoPSensor.C_in) annotation (Line(points={{520,45},{520,76},{488,76},{488,-102}}, color={28,108,200}));
  connect(ecoPSensor.C_out,ecoTSensor.C_in) annotation (Line(points={{508,-102},{512,-102}}, color={28,108,200}));
  connect(ecoTSensor.C_out,evaporator.C_cold_in) annotation (Line(points={{532,
          -102},{546,-102},{546,-148},{435,-148},{435,16.5789}},                                                                  color={28,108,200}));
  connect(evaporator.C_cold_out,evapPSensor.C_in) annotation (Line(points={{365,
          35.5263},{365,68},{380,68},{380,-102}},                                                                  color={28,108,200}));
  connect(evapPSensor.C_out,evapTSensor.C_in) annotation (Line(points={{400,-102},{404,-102}}, color={28,108,200}));
  connect(evapTSensor.C_out,superheater.C_cold_in) annotation (Line(points={{424,
          -102},{438,-102},{438,-154},{280,-154},{280,-45}},                                                                        color={28,108,200}));
  connect(superheater.C_cold_out,shPSensor.C_in) annotation (Line(points={{280,45},
          {280,60},{260,60},{260,-102}},                                                                          color={28,108,200}));
  connect(shPSensor.C_out,shTSensor.C_in) annotation (Line(points={{280,-102},{284,-102}}, color={28,108,200}));
  connect(shTSensor.C_out,steamTurbineInQSensor.C_in) annotation (Line(points={{304,-102},{327,-102},{327,-180},{500,-180}}, color={28,108,200}));
  connect(steamTurbineInQSensor.C_out,steamTurbine.C_in) annotation (Line(points={{520,-180},{540,-170}}, color={28,108,200}));
  connect(steamTurbine.C_out,stPSensor.C_in) annotation (Line(points={{580,-170},{540,-235}}, color={28,108,200}));
  connect(stPSensor.C_out,stTSensor.C_in) annotation (Line(points={{560,-235},{564,-235}}, color={28,108,200}));
  connect(stTSensor.C_out,steamCondenser.C_hot_in) annotation (Line(points={{584,-235},{440,-280}}, color={28,108,200}));
  connect(steamCondenser.C_hot_out,condPSensor.C_in) annotation (Line(points={{440,-340},{420,-395}}, color={28,108,200}));
  connect(condPSensor.C_out,condTSensor.C_in) annotation (Line(points={{440,-395},{444,-395}}, color={28,108,200}));
  connect(condTSensor.C_out,feedwaterPressureCut.C_in) annotation (Line(points={{464,-395},{480,-395},{480,-425},{18,-425},{18,-330},{30,-330}}, color={28,108,200}));
  connect(feedwaterPressureCut.C_out,feedwaterLoopBreaker.C_in) annotation (Line(points={{50,-330},{85,-330}}, color={28,108,200}));
  connect(feedwaterLoopBreaker.C_out,feedwaterPump.C_in) annotation (Line(points={{105,-330},{127,-330},{127,-320},{150,-320}}, color={28,108,200}));
  connect(coolingWaterSource.C_out,coolingWaterInTSensor.C_in) annotation (Line(points={{535,
          -370},{548,-370}},                                                                                    color={28,108,200}));
  connect(coolingWaterInTSensor.C_out,coolingWaterInPSensor.C_in) annotation (Line(points={{568,-370},{574,-370}}, color={28,108,200}));
  connect(coolingWaterInPSensor.C_out,coolingWaterInQSensor.C_in) annotation (Line(points={{594,-370},{596,-370}}, color={28,108,200}));
  connect(coolingWaterInQSensor.C_out,steamCondenser.C_cold_in) annotation (Line(points={{616,
          -370},{470,-304}},                                                                                     color={28,108,200}));
  connect(steamCondenser.C_cold_out,coolingWaterSink.C_in) annotation (Line(points={{470,
          -322},{625,-370}},                                                                                color={28,108,200}));
  connect(steamTurbine.C_W_out,generator.C_in) annotation (Line(points={{580,-154},{630,-154},{630,80},{134,80}}, color={244,125,35}));
  connect(pumpPSensor.P_sensor,pumpP) annotation (Line(points={{160,-250},{160,
          -240},{160,-230},{170,-230}},                                                color={0,0,127}));
  connect(pumpTSensor.T_sensor,pumpT) annotation (Line(points={{184,-250},{184,
          -240},{184,-230},{194,-230}},                                                color={0,0,127}));
  connect(ecoPSensor.P_sensor,ecoP) annotation (Line(points={{498,-92},{498,-87},{498,-87}}, color={0,0,127}));
  connect(ecoTSensor.T_sensor,ecoT) annotation (Line(points={{522,-92},{522,-87},{522,-87}}, color={0,0,127}));
  connect(evapPSensor.P_sensor,evapP) annotation (Line(points={{390,-92},{390,-87},{390,-87}}, color={0,0,127}));
  connect(evapTSensor.T_sensor,evapT) annotation (Line(points={{414,-92},{414,-87},{414,-87}}, color={0,0,127}));
  connect(shPSensor.P_sensor,shP) annotation (Line(points={{270,-92},{270,-87},{270,-87}}, color={0,0,127}));
  connect(shTSensor.T_sensor,shT) annotation (Line(points={{294,-92},{294,-87},{294,-87}}, color={0,0,127}));
  connect(stPSensor.P_sensor,stP) annotation (Line(points={{550,-225},{550,-216},
          {550,-205},{560,-205}},                                                  color={0,0,127}));
  connect(stTSensor.T_sensor,stT) annotation (Line(points={{574,-225},{574,-216},
          {574,-205},{584,-205}},                                                  color={0,0,127}));
  connect(condPSensor.P_sensor,condP) annotation (Line(points={{430,-385},{430,
          -376},{430,-365},{440,-365}},                                                color={0,0,127}));
  connect(condTSensor.T_sensor,condT) annotation (Line(points={{454,-385},{454,
          -376},{454,-365},{464,-365}},                                                color={0,0,127}));
  connect(steamTurbineInQSensor.Q_sensor,steamTurbineInQ) annotation (Line(points={{510,-170},{510,-135}}, color={0,0,127}));
  connect(feedwaterPumpOutQSensor.Q_sensor,feedwaterPumpOutQ) annotation (Line(points={{210,-320},{210,-285}}, color={0,0,127}));
  connect(coolingWaterInTSensor.T_sensor,coolingWaterInT) annotation (Line(points={{558,-360},{558,-340}}, color={0,0,127}));
  connect(coolingWaterInPSensor.P_sensor,coolingWaterInP) annotation (Line(points={{584,-360},{584,-340}}, color={0,0,127}));
  connect(coolingWaterInQSensor.Q_sensor,coolingWaterInQ) annotation (Line(points={{606,-360},{606,-340}}, color={0,0,127}));
  connect(economiser.Kth,economiser_Kth) annotation (Line(points={{502.4,-36},{
          480,-36},{480,-68}},                                                                color={0,0,127}));
  connect(economiser.Kfr_cold,economiser_Kfr_cold) annotation (Line(points={{502.4,
          36},{550,36},{550,-68}},                                                                      color={0,0,127}));
  connect(evaporator.Kth,evaporator_Kth) annotation (Line(points={{358,-40.2632},
          {365,-40.2632},{365,-66}},                                                          color={0,0,127}));
  connect(superheater.Kth,superheater_Kth) annotation (Line(points={{262.4,-36},
          {235,-36},{235,-66}},                                                                 color={0,0,127}));
  connect(superheater.Kfr_cold,superheater_Kfr_cold) annotation (Line(points={{262.4,
          36},{321,36},{321,-66}},                                                                        color={0,0,127}));
  connect(steamTurbine.Cst,steamTurbine_Cst) annotation (Line(points={{548,-184},
          {548,-184},{548,-125}},                                                                        color={0,0,127}));
  connect(steamTurbine.eta_is,steamTurbine_eta_is) annotation (Line(points={{556,
          -186},{592,-186},{592,-125}},                                                                        color={0,0,127}));
  connect(feedwaterPump.hn,feedwaterPump_hn) annotation (Line(points={{158,-336},
          {158,-336},{158,-365}},                                                                        color={0,0,127}));
  connect(feedwaterPump.rh,feedwaterPump_rh) annotation (Line(points={{182,-336},
          {194,-336},{194,-365}},                                                                        color={0,0,127}));
  connect(steamCondenser.Kth,steamCondenser_Kth) annotation (Line(points={{408.8,
          -304},{388,-304},{388,-345}},                                                                      color={0,0,127}));
  connect(steamCondenser.Kfr_cold,steamCondenser_Kfr_cold) annotation (Line(points={{408.8,
          -316},{393,-316},{393,-375}},                                                                                color={0,0,127}));
  connect(steamCondenser.Qv_cold_in,steamCondenser_Qv_cold_in) annotation (Line(points={{408.8,
          -310},{485,-310},{485,-345}},                                                                                    color={0,0,127}));
  connect(steamCondenser.C_incond,steamCondenser_C_incond) annotation (Line(points={{408.8,
          -298},{485,-298},{485,-375}},                                                                                color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-260,-450},{720,180}})),
    Icon(coordinateSystem(preserveAspectRatio=false,extent={{-260,-450},{720,180}})),
    experiment(StopTime=1),
    Documentation(info="<html><p>MML4 GT and single-pressure CCGT steam cycle, with locally clustered calibration parameter connectors, sensors and interfaces. Added PressureCut and LoopBreaker immediately upstream of feedwater pump. Initial guesses and parameter values are illustrative; Dymola compilation and structural balance have NOT been checked. Example values only. Built for latest MML4 SensorsControl API supplied by user; exact branch and Dymola compilation must be checked. Not translated or simulated in Dymola. Electrical power is a native MW power-sensor signal (not among the requested thermal-hydraulic display units).</p></html>"));
end CCGT_1P_MML4_direct_initialized;
