within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_reverse
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions

    // Fuel source
    input Units.SpecificEnthalpy LHV_plant(start=48130e3) "Directly assigned in combustion chamber modifiers";

  MultiFluid.HeatExchangers.Evaporator EVAP annotation (Placement(transformation(extent={{-250,-50},{-150,140}})));
  MultiFluid.HeatExchangers.Superheater RHT annotation (Placement(transformation(extent={{-350,50},{-250,-50}})));
  MultiFluid.HeatExchangers.Superheater HPSH1 annotation (Placement(transformation(extent={{-450,50},{-350,-50}})));
  MultiFluid.HeatExchangers.Superheater HPSH2 annotation (Placement(transformation(extent={{-510,-50},{-410,50}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_w_HPSH1_out_sensor(sensor_function="Calibration",
    causality="HPSH1_Kth",                                                                         T_start=453.409)
                                                                    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-400,-70})));
  Sensors_Control.WaterSteam.PressureSensor P_w_HPSH1_out_sensor(sensor_function="Calibration",
    causality="HPSH2_Kfr_cold",                                                                 P_start=114.788)
                                                                 annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-400,-100})));
  Sensors_Control.WaterSteam.PressureSensor P_w_evap_out_sensor(sensor_function="Calibration",
    causality="HPSH1_Kfr_cold",                                                                P_start=117.126) annotation (Placement(transformation(extent={{-250,110},{-270,130}})));
  MultiFluid.HeatExchangers.Economiser ECO annotation (Placement(transformation(extent={{-110,-50},{-10,50}})));
  WaterSteam.Pipes.ControlValve EVAP_controlValve annotation (Placement(transformation(extent={{-100,76},{-120,98}})));
  Sensors_Control.WaterSteam.PressureSensor P_w_eco_out_sensor(sensor_function="Calibration", causality="ECO_Kfr_cold")
                                                                                              annotation (Placement(transformation(extent={{-70,70},{-90,90}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_w_eco_out_sensor(sensor_function="Calibration",
    causality="ECO_Kth",                                                                         T_start=319.834)
                                                                                                 annotation (Placement(transformation(extent={{-130,70},{-150,90}})));
  Sensors_Control.Outline.OpeningSensor Evap_opening_sensor(sensor_function="Calibration") annotation (Placement(transformation(extent={{-120,110},{-100,130}})));
  WaterSteam.Pipes.SlideValve HPST_admission_valve annotation (Placement(transformation(extent={{-530,-204},{-510,-182}})));
  Sensors_Control.WaterSteam.PressureSensor P_HPST_in_sensor(sensor_function="Calibration",
    causality="HPST_Cst",                                                                   P_start=112.349) annotation (Placement(transformation(extent={{-500,-210},{-480,-190}})));
  WaterSteam.Pipes.SlideValve LPST_admission_valve annotation (Placement(transformation(extent={{-288,-204},{-268,-182}})));
  Sensors_Control.WaterSteam.PressureSensor P_LPST_in_sensor(sensor_function="Calibration", causality="LPST_CST")
                                                             annotation (Placement(transformation(extent={{-260,-210},{-240,-190}})));
  WaterSteam.HeatExchangers.Condenser condenser annotation (Placement(transformation(extent={{-70,-340},{30,-240}})));
  WaterSteam.Machines.FixedSpeedPump pump annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={30,-380})));
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{180,-300},{140,-260}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{140,-330},{180,-290}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_pump_out_sensor(sensor_function="Calibration",
    causality="pump_rh",                                                                        T_start=34.8939)
                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,-230})));
  Sensors_Control.WaterSteam.PressureSensor P_pump_out_sensor(sensor_function="Calibration",
    causality="pump_hn",                                                                     P_start=170.006)
                                                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,-170})));
  Sensors_Control.WaterSteam.FlowSensor Q_pump_out_sensor(sensor_function="Calibration",
    causality="evap_Kth",                                                                Q_start=49.7229)
                                                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,-200})));
  WaterSteam.Machines.FixedSpeedPump pumpRec annotation (Placement(transformation(
        extent={{-20,20},{20,-20}},
        rotation=0,
        origin={0,80})));
  Sensors_Control.WaterSteam.TemperatureSensor T_w_eco_in_sensor(sensor_function="BC", T_start=85)
                                                                                       annotation (Placement(transformation(extent={{10,-90},{-10,-70}})));
  WaterSteam.Pipes.ControlValve pumpRec_controlValve annotation (Placement(transformation(
        extent={{-10,-11},{10,11}},
        rotation=270,
        origin={86,-60})));
  WaterSteam.Machines.SteamTurbine HPST annotation (Placement(transformation(extent={{-460,-160},{-380,-240}})));
  WaterSteam.Machines.SteamTurbine LPST annotation (Placement(transformation(extent={{-220,-160},{-140,-240}})));
  Sensors_Control.WaterSteam.PressureSensor P_Cond_sensor(sensor_function="Calibration",
    causality="condenser_Kth",                                                           P_start=0.0496853)
                                                          annotation (Placement(transformation(extent={{-120,-210},{-100,-190}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_circulating_water_in_sensor(sensor_function="BC", T_start=15)
                                                                                                   annotation (Placement(transformation(extent={{140,-290},{120,-270}})));
  Sensors_Control.WaterSteam.PressureSensor P_circulating_water_in_sensor(sensor_function="BC", P_start=5)
                                                                                                annotation (Placement(transformation(extent={{110,-290},{90,-270}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_circulating_water_out_sensor(sensor_function="Calibration",
    causality="condenser_Qv_cold",
    T_start=24.9415)                                                                                                              annotation (Placement(transformation(extent={{120,-300},{140,-320}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_pumpRec_out_sensor(sensor_function="Calibration",
    causality="pumpRec_rh",                                                                        T_start=324.409)
                                                                                                   annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={80,60})));
  Sensors_Control.WaterSteam.PressureSensor P_pumpRec_out_sensor(sensor_function="Calibration",
    causality="pumpRec_hn",                                                                     P_start=195.553)
                                                                                                annotation (Placement(transformation(
        extent={{10,-11},{-10,11}},
        rotation=90,
        origin={80,30})));
  Sensors_Control.WaterSteam.FlowSensor Q_pumpRec_out_sensor(sensor_function="Unidentified", Q_start=9.37335)
                                                                                             annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={80,-20})));
  Sensors_Control.WaterSteam.TemperatureSensor T_HPST_out_sensor(sensor_function="Calibration",
    causality="HPST_eta_is",                                                                    T_start=252.806)
                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-350,-100})));
  Sensors_Control.WaterSteam.PressureSensor P_HPST_out_sensor(sensor_function="Calibration",
    causality="RHT_Kfr_cold",                                                                P_start=9.66102)
                                                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-350,-70})));
  Sensors_Control.WaterSteam.PressureSensor P_w_ReH_out_sensor(sensor_function="Calibration",
    causality="LPST_admission_valve_Cv",                                                      P_start=8.85519)
                                                               annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-300,-70})));
  Sensors_Control.WaterSteam.TemperatureSensor T_w_ReH_out_sensor(sensor_function="Calibration",
    causality="RHT_Kth",                                                                         T_start=348.684)
                                                                  annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-300,-100})));
  Sensors_Control.FlueGases.TemperatureSensor T_flue_gas_sink_sensor(T_start=339.58)
                                                                     annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Sensors_Control.FlueGases.PressureSensor P_flue_gas_sink_sensor(P_start=1)
                                                                  annotation (Placement(transformation(extent={{118,-10},{138,10}})));
  WaterSteam.Pipes.ControlValve deSH_controlValve annotation (Placement(transformation(extent={{-220,-144},{-240,-124}})));
  Power.Machines.Generator generator annotation (Placement(transformation(extent={{-388,-284},{-468,-236}})));
  Power.BoundaryConditions.Sink sink2 annotation (Placement(transformation(extent={{-490,-280},{-530,-240}})));
  Sensors_Control.Power.PowerSensor W_ST_out_sensor(sensor_function="Calibration", causality="LPST_eta_is")
                                                    annotation (Placement(transformation(extent={{-470,-270},{-490,-250}})));
  Sensors_Control.Outline.OpeningSensor pumpRec_opening_sensor(sensor_function="Calibration") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={120,-60})));
  Sensors_Control.FlueGases.TemperatureSensor turbine_T_out_sensor(sensor_function="BC", T_start=640) annotation (Placement(transformation(extent={{-570,-10},{-550,10}})));
  Sensors_Control.FlueGases.PressureSensor turbine_P_out_sensor(sensor_function="BC", P_start=1.11116) annotation (Placement(transformation(extent={{-520,-10},{-500,10}})));
  Sensors_Control.FlueGases.FlowSensor Q_source_air_sensor(sensor_function="BC", Q_start=500) annotation (Placement(transformation(extent={{-1020,-10},{-1000,10}})));
  Sensors_Control.Outline.OpeningSensor deSH_opening_sensor(sensor_function="Calibration")
                                                            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-230,-100})));
  Sensors_Control.WaterSteam.TemperatureSensor T_w_HPSH2_out_sensor(sensor_function="BC", T_start=566.5) annotation (Placement(transformation(extent={{-470,90},{-490,110}})));
  Sensors_Control.WaterSteam.PressureSensor P_w_HPSH2_out_sensor(sensor_function="Calibration",
    causality="HPST_admission_valve_Cv",                                                        P_start=113.3) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-510,100})));
  Utilities.Interfaces.RealInput turbine_T_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-560,20}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput turbine_P_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-510,20}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput Q_source_air annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-1010,20}),iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput T_w_HPSH2_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-480,120}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_w_HPSH2_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-510,120}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_HPST_in annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-490,-180}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_w_evap_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-260,140}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput T_HPST_out
                                           annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-370,-100}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_HPST_out
                                           annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-370,-70}),  iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput T_w_ReH_out
                                           annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={-280,-100}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_w_ReH_out
                                           annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={-280,-70}),  iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_LPST_in annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-250,-180}), iconTransformation(extent={{-430,-154},{-390,-114}})));
  Utilities.Interfaces.RealInput P_Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-110,-180}), iconTransformation(extent={{-604,-234},{-564,-194}})));
  Utilities.Interfaces.RealInput T_circulating_water_in annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={130,-260}), iconTransformation(extent={{-604,-234},{-564,-194}})));
  Utilities.Interfaces.RealInput P_circulating_water_in annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={100,-260}), iconTransformation(extent={{-604,-234},{-564,-194}})));
  Utilities.Interfaces.RealInput T_circulating_water_out annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=270,
        origin={130,-330}), iconTransformation(extent={{-604,-234},{-564,-194}})));
  Utilities.Interfaces.RealInput T_pump_out annotation (Placement(transformation(extent={{56,-234},{64,-226}}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput Q_pump_out annotation (Placement(transformation(extent={{56,-204},{64,-196}}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput P_pump_out annotation (Placement(transformation(extent={{56,-174},{64,-166}}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput T_w_eco_in annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,-60}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput T_flue_gas_sink annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={100,20}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput P_flue_gas_sink annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={128,20}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput P_pumpRec_out annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={60,30}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput T_pumpRec_out annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={60,60}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput Q_pumpRec_out annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={60,-20}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput pumpRec_opening(start=21.7) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={140,-60}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput Evap_opening(start=11.9862) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-110,140}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput deSH_opening(start=16.4941) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-230,-80}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput T_w_HPSH1_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-420,-70}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_w_HPSH1_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-420,-100}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_HPST_in1(start=64.8383)
                                           annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-480,-240}), iconTransformation(extent={{-696,-10},{-656,30}})));
  Utilities.Interfaces.RealInput P_w_eco_out(start=11.9862) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,100}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealInput T_w_eco_out(start=11.9862) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-140,100}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={80,-350})));
  Sensors_Control.WaterSteam.FlowSensor Q_deSH_sensor(
    sensor_function="Calibration",
    causality="HPSH2_Kth",
    Q_start=2.2273,
    signal_unit="kg/s") annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-170,-140})));
  Utilities.Interfaces.RealInput Q_deSH annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-170,-120}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  FlueGases.Machines.GasTurbine gasTurbine annotation (Placement(transformation(extent={{-660,-40},{-580,40}})));
  MultiFluid.Machines.CombustionChamber combustionChamber(LHV=LHV_plant) annotation (Placement(transformation(extent={{-720,-20},{-680,20}})));
  Sensors_Control.FlueGases.TemperatureSensor compressor_T_out_sensor(sensor_function="Calibration",
    causality="compressor_eta_is",                                                                   T_start=450) annotation (Placement(transformation(extent={{-780,-10},{-760,10}})));
  Sensors_Control.FlueGases.PressureSensor compressor_P_out_sensor(sensor_function="Calibration",
    causality="compressor_tau",                                                                   P_start=16.9887) annotation (Placement(transformation(extent={{-750,-10},{-730,10}})));
  Sensors_Control.Fuel.TemperatureSensor T_fuel_source_sensor(sensor_function="BC", T_start=156) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-700,-40})));
  Sensors_Control.Fuel.PressureSensor P_fuel_source_sensor(sensor_function="BC", P_start=30) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-700,-70})));
  Sensors_Control.Fuel.FlowSensor Q_fuel_source_sensor(sensor_function="Calibration",
    causality="CC_eta",                                                               Q_start=10.5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-700,-100})));
  Fuel.BoundaryConditions.Chromatograph
                                 chromatograph(signal_unit="%mass")
                                             annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-700,-130})));
  Utilities.Interfaces.RealInput compressor_T_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-770,20}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput compressor_P_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-740,20}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput T_fuel_source annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-720,-40}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput compressor_T_out2 annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-720,-70}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput Q_fuel_source annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-720,-100}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Power.Machines.Generator GTgenerator annotation (Placement(transformation(extent={{-580,56},{-660,104}})));
  Power.BoundaryConditions.Sink power_GT_sink annotation (Placement(transformation(extent={{-680,60},{-720,100}})));
  Sensors_Control.Power.PowerSensor W_GT_out_sensor(sensor_function="Calibration", causality="GT_trubine_eta_is")
                                                    annotation (Placement(transformation(extent={{-660,70},{-680,90}})));
  Utilities.Interfaces.RealInput W_GT_out(start=148.912) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-670,100}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  FlueGases.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-880,-32},{-800,32}})));
  Utilities.Interfaces.RealInput T_source_air annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-1076,60}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Utilities.Interfaces.RealInput P_source_air annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-1060,60}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  Sensors_Control.FlueGases.PressureSensor P_filter_out_sensor(sensor_function="Calibration",
    causality="air_filter_Kfr",                                                               P_start=0.899403) annotation (Placement(transformation(extent={{-940,-10},{-920,10}})));
  Utilities.Interfaces.RealInput P_filter_out annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-930,20}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  FlueGases.Machines.InletGuideVanes inletGuideVanes annotation (Placement(transformation(extent={{-920,-20},{-880,20}})));
  FlueGases.Pipes.Filter filter annotation (Placement(transformation(extent={{-990,-20},{-950,20}})));
  Sensors_Control.Outline.OpeningSensor openingSensor(sensor_function="Calibration") annotation (Placement(transformation(extent={{-910,30},{-890,50}})));
  Utilities.Interfaces.RealInput IGV_angle(start=100) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-900,60}), iconTransformation(extent={{-1036,0},{-996,40}})));
  Utilities.Interfaces.RealInput H_source           annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-1044,60}), iconTransformation(extent={{-800,-194},{-760,-154}})));
  FlueGases.BoundaryConditions.Flue_gas_stack flue_gas_stack_2D annotation (Placement(transformation(extent={{130,-10},{230,130}})));
  Utilities.Interfaces.GasCompositionInput composition annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-700,-162})));
  FlueGases.BoundaryConditions.GT_louvers gT_louvers(
    T_signal_unit="degC",
    T_start=25,
    P_start=1,
    H_start=50)                                      annotation (Placement(transformation(extent={{-1100,-40},{-1020,40}})));
  Utilities.Interfaces.RealOutput air_filter_Kfr annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-970,40}), iconTransformation(extent={{-1072,-32},{-1052,-12}})));
  Utilities.Interfaces.RealOutput compressor_tau annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-864,-40}), iconTransformation(extent={{-1072,-32},{-1052,-12}})));
  Utilities.Interfaces.RealOutput compressor_eta_is annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-840,-40}), iconTransformation(extent={{-1072,-32},{-1052,-12}})));
  Utilities.Interfaces.RealOutput CC_eta annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-708,40}), iconTransformation(extent={{-1072,-32},{-1052,-12}})));
  Utilities.Interfaces.RealOutput GT_trubine_eta_is annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-628,-60}), iconTransformation(extent={{-932,-38},{-912,-18}})));
  Utilities.Interfaces.RealOutput HPSH2_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-500,-40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput HPSH1_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-432,60}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput RHT_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-340,40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput ECO_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-100,-40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput LPST_eta_is annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-188,-160}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput LPST_CST annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-204,-160}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput HPST_Cst annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-444,-160}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput HPST_eta_is annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-428,-160}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput condenser_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-100,-280}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput condenser_Qv_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-100,-290}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput LPST_admission_valve_Cv annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-290,-190}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput RHT_Kfr_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-340,-40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput HPSH1_Kfr_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-430,-62}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput HPSH2_Kfr_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-500,40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput pumpRec_rh annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={12,120}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput pumpRec_hn annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-12,120}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput pump_hn annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={18,-412}),iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput pump_rh annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=270,
        origin={42,-412}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput evap_Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-260,-40}), iconTransformation(extent={{-686,-116},{-666,-96}})));
  Utilities.Interfaces.RealOutput ECO_Kfr_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-100,40}), iconTransformation(extent={{-178,-276},{-138,-236}})));
  Utilities.Interfaces.RealOutput HPST_admission_valve_Cv annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-532,-188}), iconTransformation(extent={{-686,-116},{-666,-96}})));
equation

  /* Hypotheses */
  // Condenser
  condenser.C_incond = 0;
  condenser.Kfr_cold = 0;
  // Combustion chamber
  combustionChamber.Kfr = 0;

  connect(deSH_controlValve.C_in, Q_deSH_sensor.C_out) annotation (Line(
      points={{-220,-140},{-180,-140}},
      color={28,108,200},
      thickness=0.5));
  connect(Q_deSH_sensor.C_in, P_pump_out_sensor.C_out) annotation (Line(
      points={{-160,-140},{80,-140},{80,-160}},
      color={28,108,200},
      thickness=0.5));
  connect(deSH_controlValve.C_out, HPSH2.C_cold_in) annotation (Line(
      points={{-240,-140},{-420,-140},{-420,-120},{-460,-120},{-460,-50}},
      color={28,108,200},
      thickness=0.5));
  connect(EVAP.C_hot_in, RHT.C_hot_out) annotation (Line(
      points={{-240,0},{-280,0}},
      color={95,95,95},
      thickness=1));
  connect(RHT.C_hot_in, HPSH1.C_hot_out) annotation (Line(
      points={{-320,0},{-380,0}},
      color={95,95,95},
      thickness=1));
  connect(HPSH1.C_hot_in, HPSH2.C_hot_out) annotation (Line(
      points={{-420,0},{-440,0}},
      color={95,95,95},
      thickness=1));
  connect(HPSH1.C_cold_out, T_w_HPSH1_out_sensor.C_in) annotation (Line(
      points={{-400,-50},{-400,-60}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(T_w_HPSH1_out_sensor.C_out, P_w_HPSH1_out_sensor.C_in) annotation (Line(
      points={{-400,-80},{-400,-90}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_HPSH1_out_sensor.C_out, HPSH2.C_cold_in) annotation (Line(
      points={{-400,-110},{-400,-120},{-460,-120},{-460,-50}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(EVAP.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
      points={{-235,120},{-250,120}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_evap_out_sensor.C_out, HPSH1.C_cold_in) annotation (Line(
      points={{-270,120},{-400,120},{-400,50}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(EVAP.C_hot_out, ECO.C_hot_in) annotation (Line(
      points={{-160,0},{-80,0}},
      color={95,95,95},
      thickness=1));
  connect(P_w_eco_out_sensor.C_out, EVAP_controlValve.C_in) annotation (Line(
      points={{-90,80},{-100,80},{-100,80.4}},
      color={28,108,200},
      thickness=1));
  connect(EVAP_controlValve.C_out, T_w_eco_out_sensor.C_in) annotation (Line(
      points={{-120,80.4},{-120,80},{-130,80}},
      color={28,108,200},
      thickness=1));
  connect(T_w_eco_out_sensor.C_out, EVAP.C_cold_in) annotation (Line(
      points={{-150,80},{-165,80}},
      color={28,108,200},
      thickness=1));
  connect(EVAP_controlValve.Opening, Evap_opening_sensor.Opening) annotation (Line(points={{-110,98},{-110,109.8}}, color={0,0,127}));
  connect(HPST_admission_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
      points={{-510,-200},{-500,-200}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(LPST_admission_valve.C_out, P_LPST_in_sensor.C_in) annotation (Line(
      points={{-268,-200},{-260,-200}},
      color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(condenser.C_hot_out, pump.C_in) annotation (Line(
      points={{-20,-340},{-20,-380},{10,-380}},
      color={28,108,200},
      thickness=1));
  connect(ECO.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
      points={{-60,50},{-60,80},{-70,80}},
      color={28,108,200},
      thickness=1));
  connect(Q_pump_out_sensor.C_in, T_pump_out_sensor.C_out) annotation (Line(
      points={{80,-210},{80,-220}},
      color={28,108,200},
      thickness=1));
  connect(P_pump_out_sensor.C_in, Q_pump_out_sensor.C_out) annotation (Line(
      points={{80,-180},{80,-190}},
      color={28,108,200},
      thickness=1));
  connect(pumpRec.C_in, P_w_eco_out_sensor.C_in) annotation (Line(
      points={{-20,80},{-70,80}},
      color={28,108,200},
      thickness=1));
  connect(ECO.C_cold_in, T_w_eco_in_sensor.C_out) annotation (Line(
      points={{-60,-50},{-60,-80},{-10,-80}},
      color={28,108,200},
      thickness=1));
  connect(T_w_eco_in_sensor.C_in, P_pump_out_sensor.C_out) annotation (Line(
      points={{10,-80},{80,-80},{80,-160}},
      color={28,108,200},
      thickness=1));
  connect(pumpRec_controlValve.C_out, P_pump_out_sensor.C_out) annotation (Line(
      points={{79.4,-70},{80,-70},{80,-160}},
      color={28,108,200},
      thickness=1));
  connect(P_HPST_in_sensor.C_out, HPST.C_in) annotation (Line(
      points={{-480,-200},{-460,-200}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_LPST_in_sensor.C_out, LPST.C_in) annotation (Line(
      points={{-240,-200},{-220,-200}},
      color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LPST.C_out, P_Cond_sensor.C_in) annotation (Line(
      points={{-140,-200},{-120,-200}},
      color={28,108,200},
      thickness=1));
  connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(
      points={{-100,-200},{-20,-200},{-20,-240}},
      color={28,108,200},
      thickness=1));
  connect(T_circulating_water_in_sensor.C_in, source.C_out) annotation (Line(
      points={{140,-280},{150,-280}},
      color={28,108,200},
      thickness=0.5));
  connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (Line(
      points={{30,-280},{90,-280}},
      color={28,108,200},
      thickness=0.5));
  connect(P_circulating_water_in_sensor.C_in, T_circulating_water_in_sensor.C_out) annotation (Line(
      points={{110,-280},{120,-280}},
      color={28,108,200},
      thickness=0.5));
  connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in) annotation (Line(
      points={{30,-310},{120,-310}},
      color={28,108,200},
      thickness=0.5));
  connect(T_circulating_water_out_sensor.C_out, sink.C_in) annotation (Line(
      points={{140,-310},{150,-310}},
      color={28,108,200},
      thickness=0.5));
  connect(pumpRec.C_out, T_pumpRec_out_sensor.C_in) annotation (Line(
      points={{20,80},{80,80},{80,70}},
      color={28,108,200},
      thickness=1));
  connect(T_pumpRec_out_sensor.C_out, P_pumpRec_out_sensor.C_in) annotation (Line(
      points={{80,50},{80,40}},
      color={28,108,200},
      thickness=1));
  connect(P_pumpRec_out_sensor.C_out, Q_pumpRec_out_sensor.C_in) annotation (Line(
      points={{80,20},{80,-10}},
      color={28,108,200},
      thickness=1));
  connect(Q_pumpRec_out_sensor.C_out, pumpRec_controlValve.C_in) annotation (Line(
      points={{80,-30},{80,-50},{79.4,-50}},
      color={28,108,200},
      thickness=1));
  connect(HPST.C_out, T_HPST_out_sensor.C_in) annotation (Line(
      points={{-380,-200},{-350,-200},{-350,-110}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_HPST_out_sensor.C_out, RHT.C_cold_in) annotation (Line(
      points={{-350,-60},{-350,80},{-300,80},{-300,50}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(RHT.C_cold_out, P_w_ReH_out_sensor.C_in) annotation (Line(points={{-300,-50},{-300,-60}}, color={28,108,200},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(P_w_ReH_out_sensor.C_out, T_w_ReH_out_sensor.C_in) annotation (Line(
      points={{-300,-80},{-300,-90}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(T_w_ReH_out_sensor.C_out, LPST_admission_valve.C_in) annotation (Line(
      points={{-300,-110},{-300,-200},{-288,-200}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(ECO.C_hot_out, T_flue_gas_sink_sensor.C_in) annotation (Line(
      points={{-40,0},{90,0}},
      color={95,95,95},
      thickness=1));
  connect(T_flue_gas_sink_sensor.C_out, P_flue_gas_sink_sensor.C_in) annotation (Line(
      points={{110,0},{118,0}},
      color={95,95,95},
      thickness=1));
  connect(P_HPST_out_sensor.C_in, T_HPST_out_sensor.C_out) annotation (Line(
      points={{-350,-80},{-350,-90}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(HPST.C_W_out, LPST.C_W_out) annotation (Line(
      points={{-380,-233.6},{-380,-260},{-140,-260},{-140,-233.6}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(generator.C_in, LPST.C_W_out) annotation (Line(
      points={{-403.2,-260},{-140,-260},{-140,-233.6}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(generator.C_out, W_ST_out_sensor.C_in) annotation (Line(
      points={{-456,-260},{-470,-260}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(W_ST_out_sensor.C_out, sink2.C_in) annotation (Line(
      points={{-489.8,-260},{-500,-260}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(pumpRec_controlValve.Opening, pumpRec_opening_sensor.Opening) annotation (Line(points={{97,-60},{109.8,-60}}, color={0,0,127}));
  connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out) annotation (Line(points={{-520,0},{-550,0}}, color={95,95,95},
      thickness=1));
  connect(deSH_controlValve.Opening, deSH_opening_sensor.Opening) annotation (Line(points={{-230,-124},{-230,-110.2}}, color={0,0,127}));
  connect(HPSH2.C_cold_out, T_w_HPSH2_out_sensor.C_in) annotation (Line(
      points={{-460,50},{-460,100},{-470,100}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(T_w_HPSH2_out_sensor.C_out, P_w_HPSH2_out_sensor.C_in) annotation (Line(
      points={{-490,100},{-500,100}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_HPSH2_out_sensor.C_out, HPST_admission_valve.C_in) annotation (Line(
      points={{-520,100},{-540,100},{-540,-200},{-530,-200}},
      color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(turbine_T_out_sensor.T_sensor, turbine_T_out) annotation (Line(points={{-560,10},{-560,20}}, color={0,0,127}));
  connect(turbine_P_out_sensor.P_sensor, turbine_P_out) annotation (Line(points={{-510,10},{-510,20}}, color={0,0,127}));
  connect(Q_source_air_sensor.Q_sensor, Q_source_air) annotation (Line(points={{-1010,10},{-1010,20}},
                                                                                                     color={0,0,127}));
  connect(T_w_HPSH2_out_sensor.T_sensor, T_w_HPSH2_out) annotation (Line(points={{-480,110},{-480,120}}, color={0,0,127}));
  connect(P_w_HPSH2_out_sensor.P_sensor, P_w_HPSH2_out) annotation (Line(points={{-510,110},{-510,120}}, color={0,0,127}));
  connect(P_HPST_in_sensor.P_sensor, P_HPST_in) annotation (Line(points={{-490,-190},{-490,-180}}, color={0,0,127}));
  connect(P_w_evap_out_sensor.P_sensor, P_w_evap_out) annotation (Line(points={{-260,130},{-260,140}}, color={0,0,127}));
  connect(T_HPST_out_sensor.T_sensor, T_HPST_out) annotation (Line(points={{-360,-100},{-370,-100}}, color={0,0,127}));
  connect(P_HPST_out_sensor.P_sensor, P_HPST_out) annotation (Line(points={{-360,-70},{-370,-70}}, color={0,0,127}));
  connect(T_w_ReH_out_sensor.T_sensor, T_w_ReH_out) annotation (Line(points={{-290,-100},{-280,-100}}, color={0,0,127}));
  connect(P_w_ReH_out_sensor.P_sensor, P_w_ReH_out) annotation (Line(points={{-290,-70},{-280,-70}}, color={0,0,127}));
  connect(P_LPST_in_sensor.P_sensor, P_LPST_in) annotation (Line(points={{-250,-190},{-250,-180}}, color={0,0,127}));
  connect(P_Cond_sensor.P_sensor, P_Cond) annotation (Line(points={{-110,-190},{-110,-180}}, color={0,0,127}));
  connect(T_circulating_water_in_sensor.T_sensor, T_circulating_water_in) annotation (Line(points={{130,-270},{130,-260}}, color={0,0,127}));
  connect(P_circulating_water_in_sensor.P_sensor, P_circulating_water_in) annotation (Line(points={{100,-270},{100,-260}}, color={0,0,127}));
  connect(T_circulating_water_out_sensor.T_sensor, T_circulating_water_out) annotation (Line(points={{130,-320},{130,-330}}, color={0,0,127}));
  connect(T_pump_out_sensor.T_sensor, T_pump_out) annotation (Line(points={{70,-230},{60,-230}}, color={0,0,127}));
  connect(Q_pump_out_sensor.Q_sensor, Q_pump_out) annotation (Line(points={{70,-200},{60,-200}}, color={0,0,127}));
  connect(P_pump_out_sensor.P_sensor, P_pump_out) annotation (Line(points={{70,-170},{60,-170}}, color={0,0,127}));
  connect(T_w_eco_in_sensor.T_sensor, T_w_eco_in) annotation (Line(points={{0,-70},{0,-60}}, color={0,0,127}));
  connect(T_flue_gas_sink, T_flue_gas_sink_sensor.T_sensor) annotation (Line(points={{100,20},{100,10}}, color={0,0,127}));
  connect(P_flue_gas_sink_sensor.P_sensor, P_flue_gas_sink) annotation (Line(points={{128,10},{128,20}}, color={0,0,127}));
  connect(P_pumpRec_out_sensor.P_sensor, P_pumpRec_out) annotation (Line(points={{69,30},{60,30}}, color={0,0,127}));
  connect(T_pumpRec_out_sensor.T_sensor, T_pumpRec_out) annotation (Line(points={{70,60},{60,60}}, color={0,0,127}));
  connect(Q_pumpRec_out_sensor.Q_sensor, Q_pumpRec_out) annotation (Line(points={{70,-20},{60,-20}}, color={0,0,127}));
  connect(pumpRec_opening_sensor.opening_sensor, pumpRec_opening) annotation (Line(points={{130.2,-60},{140,-60}}, color={0,0,127}));
  connect(Evap_opening_sensor.opening_sensor, Evap_opening) annotation (Line(points={{-110,130.2},{-110,140}}, color={0,0,127}));
  connect(deSH_opening_sensor.opening_sensor, deSH_opening) annotation (Line(points={{-230,-89.8},{-230,-80}}, color={0,0,127}));
  connect(T_w_HPSH1_out_sensor.T_sensor, T_w_HPSH1_out) annotation (Line(points={{-410,-70},{-420,-70}}, color={0,0,127}));
  connect(P_w_HPSH1_out_sensor.P_sensor, P_w_HPSH1_out) annotation (Line(points={{-410,-100},{-420,-100}}, color={0,0,127}));
  connect(W_ST_out_sensor.W_sensor, P_HPST_in1) annotation (Line(points={{-480,-250},{-480,-240}}, color={0,0,127}));
  connect(P_w_eco_out_sensor.P_sensor, P_w_eco_out) annotation (Line(points={{-80,90},{-80,100}}, color={0,0,127}));
  connect(T_w_eco_out_sensor.T_sensor, T_w_eco_out) annotation (Line(points={{-140,90},{-140,100}}, color={0,0,127}));
  connect(T_pump_out_sensor.C_in, loopBreaker.C_out) annotation (Line(
      points={{80,-240},{80,-340}},
      color={28,108,200},
      thickness=1));
  connect(loopBreaker.C_in, pump.C_out) annotation (Line(
      points={{80,-360},{80,-380},{50,-380}},
      color={28,108,200},
      thickness=1));
  connect(Q_deSH_sensor.Q_sensor, Q_deSH) annotation (Line(points={{-170,-130},{-170,-120}}, color={0,0,127}));
  connect(turbine_T_out_sensor.C_in, gasTurbine.C_out) annotation (Line(
      points={{-570,0},{-580,0}},
      color={95,95,95},
      thickness=1));
  connect(HPSH2.C_hot_in, turbine_P_out_sensor.C_out) annotation (Line(points={{-480,0},{-500,0}}, color={95,95,95},
      thickness=1));
  connect(combustionChamber.outlet, gasTurbine.C_in) annotation (Line(points={{-680,0},{-660,0}}, color={95,95,95},
      thickness=1));
  connect(combustionChamber.inlet, compressor_P_out_sensor.C_out) annotation (Line(points={{-720,0},{-730,0}}, color={95,95,95},
      thickness=1));
  connect(compressor_P_out_sensor.C_in, compressor_T_out_sensor.C_out) annotation (Line(points={{-750,0},{-760,0}}, color={95,95,95},
      thickness=1));
  connect(combustionChamber.inlet1, T_fuel_source_sensor.C_out) annotation (Line(
      points={{-700,-20},{-700,-30}},
      color={213,213,0},
      thickness=1));
  connect(Q_fuel_source_sensor.C_in, chromatograph.C_out) annotation (Line(
      points={{-700,-110},{-700,-120}},
      color={213,213,0},
      thickness=1));
  connect(Q_fuel_source_sensor.C_out, P_fuel_source_sensor.C_in) annotation (Line(
      points={{-700,-90},{-700,-80}},
      color={213,213,0},
      thickness=1));
  connect(P_fuel_source_sensor.C_out, T_fuel_source_sensor.C_in) annotation (Line(
      points={{-700,-60},{-700,-50}},
      color={213,213,0},
      thickness=1));
  connect(compressor_T_out_sensor.T_sensor, compressor_T_out) annotation (Line(points={{-770,10},{-770,20}}, color={0,0,127}));
  connect(compressor_P_out_sensor.P_sensor, compressor_P_out) annotation (Line(points={{-740,10},{-740,20}}, color={0,0,127}));
  connect(T_fuel_source_sensor.T_sensor, T_fuel_source) annotation (Line(points={{-710,-40},{-720,-40}}, color={0,0,127}));
  connect(P_fuel_source_sensor.P_sensor, compressor_T_out2) annotation (Line(points={{-710,-70},{-720,-70}}, color={0,0,127}));
  connect(Q_fuel_source_sensor.Q_sensor, Q_fuel_source) annotation (Line(points={{-710,-100},{-720,-100}}, color={0,0,127}));
  connect(GTgenerator.C_out, W_GT_out_sensor.C_in) annotation (Line(
      points={{-648,80},{-660,80}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(W_GT_out_sensor.C_out, power_GT_sink.C_in) annotation (Line(
      points={{-679.8,80},{-690,80}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(GTgenerator.C_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-595.2,80},{-580,80},{-580,40}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(W_GT_out_sensor.W_sensor, W_GT_out) annotation (Line(points={{-670,90},{-670,100}}, color={0,0,127}));
  connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(
      points={{-800,0},{-780,0}},
      color={95,95,95},
      thickness=1));
  connect(airCompressor.C_W_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-800,24},{-800,60},{-580,60},{-580,40}},
      color={244,125,35},
      smooth=Smooth.Bezier,
      thickness=1));
  connect(P_filter_out_sensor.P_sensor, P_filter_out) annotation (Line(points={{-930,10},{-930,20}}, color={0,0,127}));
  connect(P_filter_out_sensor.C_out, inletGuideVanes.C_in) annotation (Line(
      points={{-920,0},{-910,0}},
      color={95,95,95},
      thickness=1));
  connect(inletGuideVanes.C_out, airCompressor.C_in) annotation (Line(
      points={{-890,0},{-880,0}},
      color={95,95,95},
      thickness=1));
  connect(Q_source_air_sensor.C_out, filter.C_in) annotation (Line(
      points={{-1000,0},{-990,0}},
      color={95,95,95},
      thickness=1));
  connect(filter.C_out, P_filter_out_sensor.C_in) annotation (Line(
      points={{-950,0},{-940,0}},
      color={95,95,95},
      thickness=1));
  connect(inletGuideVanes.Opening, openingSensor.Opening) annotation (Line(points={{-900,16},{-900,29.8}}, color={0,0,127}));
  connect(openingSensor.opening_sensor, IGV_angle) annotation (Line(points={{-900,50.2},{-900,60}}, color={0,0,127}));
  connect(P_flue_gas_sink_sensor.C_out, flue_gas_stack_2D.C_in) annotation (Line(
      points={{138,0},{155,0}},
      color={95,95,95},
      thickness=1));
  connect(composition, chromatograph.composition) annotation (Line(
      points={{-700,-162},{-700,-145.2}},
      color={211,211,0},
      thickness=0.5));
  connect(Q_source_air_sensor.C_in, gT_louvers.C_out) annotation (Line(
      points={{-1020,0},{-1036,0}},
      color={95,95,95},
      thickness=1));
  connect(gT_louvers.P, P_source_air) annotation (Line(points={{-1060,40},{-1060,60}}, color={0,0,127}));
  connect(gT_louvers.H, H_source) annotation (Line(points={{-1044,40},{-1044,60}}, color={0,0,127}));
  connect(gT_louvers.T, T_source_air) annotation (Line(points={{-1076,40},{-1076,60}}, color={0,0,127}));
  connect(filter.Kfr, air_filter_Kfr) annotation (Line(points={{-970,8},{-970,40}}, color={0,0,127}));
  connect(airCompressor.tau, compressor_tau) annotation (Line(points={{-864,-24},{-864,-40}}, color={0,0,127}));
  connect(airCompressor.eta_is, compressor_eta_is) annotation (Line(points={{-840,-20},{-840,-40}}, color={0,0,127}));
  connect(combustionChamber.eta, CC_eta) annotation (Line(points={{-708,20},{-708,40}}, color={0,0,127}));
  connect(gasTurbine.eta_is, GT_trubine_eta_is) annotation (Line(points={{-628,-32},{-628,-60}}, color={0,0,127}));
  connect(HPSH2.Kth, HPSH2_Kth) annotation (Line(points={{-482,-40},{-500,-40}}, color={0,0,127}));
  connect(RHT.Kth, RHT_Kth) annotation (Line(points={{-322,40},{-340,40}}, color={0,0,127}));
  connect(RHT_Kth, RHT_Kth) annotation (Line(points={{-340,40},{-340,40}}, color={0,0,127}));
  connect(HPSH1_Kth, HPSH1.Kth) annotation (Line(points={{-432,60},{-432,40},{-422,40}}, color={0,0,127}));
  connect(ECO.Kth, ECO_Kth) annotation (Line(points={{-82,-40},{-100,-40}}, color={0,0,127}));
  connect(LPST.eta_is, LPST_eta_is) annotation (Line(points={{-188,-168},{-188,-160}}, color={0,0,127}));
  connect(LPST.Cst, LPST_CST) annotation (Line(points={{-204,-172},{-204,-160}}, color={0,0,127}));
  connect(HPST.Cst, HPST_Cst) annotation (Line(points={{-444,-172},{-444,-160}}, color={0,0,127}));
  connect(HPST.eta_is, HPST_eta_is) annotation (Line(points={{-428,-168},{-428,-160}}, color={0,0,127}));
  connect(condenser.Kth, condenser_Kth) annotation (Line(points={{-72,-280},{-100,-280}}, color={0,0,127}));
  connect(condenser.Qv_cold_in, condenser_Qv_cold) annotation (Line(points={{-72,-290},{-100,-290}}, color={0,0,127}));
  connect(LPST_admission_valve_Cv, LPST_admission_valve.Cv) annotation (Line(points={{-290,-190},{-288,-189},{-282,-189}}, color={0,0,127}));
  connect(RHT.Kfr_cold, RHT_Kfr_cold) annotation (Line(points={{-322,-40},{-340,-40}}, color={0,0,127}));
  connect(HPSH1.Kfr_cold, HPSH1_Kfr_cold) annotation (Line(points={{-422,-40},{-430,-40},{-430,-62}}, color={0,0,127}));
  connect(HPSH2.Kfr_cold, HPSH2_Kfr_cold) annotation (Line(points={{-482,40},{-500,40}}, color={0,0,127}));
  connect(pumpRec.rh, pumpRec_rh) annotation (Line(points={{12,96},{12,120}}, color={0,0,127}));
  connect(pumpRec.hn, pumpRec_hn) annotation (Line(points={{-12,96},{-12,120}}, color={0,0,127}));
  connect(pump.hn, pump_hn) annotation (Line(points={{18,-396},{18,-412}},          color={0,0,127}));
  connect(pump.rh, pump_rh) annotation (Line(points={{42,-396},{42,-412}},           color={0,0,127}));
  connect(EVAP.Kth, evap_Kth) annotation (Line(points={{-242,-40},{-260,-40}}, color={0,0,127}));
  connect(ECO.Kfr_cold, ECO_Kfr_cold) annotation (Line(points={{-82,40},{-100,40}}, color={0,0,127}));
  connect(HPST_admission_valve.Cv, HPST_admission_valve_Cv) annotation (Line(points={{-524,-189},{-524,-188},{-532,-188}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-1120,-420},{220,160}})),
                                                              Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-1120,-420},{220,160}})));
end MetroscopiaCCGT_reverse;
