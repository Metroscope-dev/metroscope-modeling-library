within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_direct_mml4
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions

    // Fuel source
    input Units.SpecificEnthalpy LHV_plant(start=48130e3) "Directly assigned in combustion chamber modifiers";

  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Economiser economiser(
      QCp_max_side="hot")
    annotation (Placement(transformation(extent={{71,-29.5},{129,29.5}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink flue_gas_sink
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={222,224})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_w_eco_out_sensor(sensor_function="Calibration", causality="Eco_Kth")
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={12,34})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_w_eco_out_sensor(sensor_function="Calibration", causality="Eco_Kfr")
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={56,34})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Evaporator evaporator(faulty=false)
    annotation (Placement(transformation(extent={{-36,-30},{20,30}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_w_evap_out_sensor(sensor_function="Calibration", causality="SH1_Kfr")
    annotation (Placement(transformation(extent={{-34,28},{-46,40}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater1(
      QCp_max_side="hot")
    annotation (Placement(transformation(extent={{-186,-30},{-126,30}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   T_w_HPSH1_out_sensor(sensor_function="Calibration", causality="SH1_Kth")
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={-184,34})));
  Sensors_Control.WaterSteam.PressureSensor                   P_w_HPSH1_out_sensor(sensor_function="Calibration", causality="SH2_Kfr")
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={-208,34})));
  WaterSteam.Pipes.SlideValve                             HPST_control_valve
    annotation (Placement(transformation(extent={{-223.25,176.738},{-206.75,194.677}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_HPST_in_sensor(sensor_function="Calibration", causality="HPST_Cst")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-194,180})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPsteamTurbine annotation (Placement(transformation(extent={{-180,164},{-146,196}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_HPST_out_sensor(sensor_function="Calibration", causality="RHT_Kfr")
    annotation (Placement(transformation(extent={{-134,174},{-122,186}})));
  MetroscopeModelingLibrary.Sensors_Control.Power.PowerSensor W_ST_out_sensor(sensor_function="Calibration", causality="LPST_eta_is")
    annotation (Placement(transformation(extent={{90,314},{102,326}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{40,185.778},{80,217.778}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_circulating_water_out_sensor(sensor_function="Calibration", causality="Cond_Qv")
    annotation (Placement(transformation(extent={{105,195},{115,205}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source circulating_water_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-40,200})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink circulating_water_sink
    annotation (Placement(transformation(extent={{122,192},{138,208}})));
  WaterSteam.Machines.FixedSpeedPump                 pump annotation (Placement(
        transformation(
        extent={{-7,-7},{7,7}},
        origin={110,160},
        rotation=0)));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_pump_out_sensor(sensor_function="Calibration", causality="rh")
    annotation (Placement(transformation(
        extent={{5,5},{-5,-5}},
        rotation=180,
        origin={130,160})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_pump_out_sensor(sensor_function="Calibration", causality="hn")
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={150,160})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={180,54})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.FlowSensor Q_pump_out_sensor(sensor_function="Calibration", causality="Evap_Kth")
    annotation (Placement(transformation(extent={{165,155},{175,165}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor(h_out(
        start=7e5))
    annotation (Placement(transformation(extent={{-524,-14},{-496,14}})));
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine gasTurbine(
    eta_is(start=0.73),
    eta_mech(start=0.9),
    h_out(start=0.5e6))
    annotation (Placement(transformation(extent={{-412,-16},{-380,16}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_power
    annotation (Placement(transformation(extent={{-332,90},{-312,110}})));
  MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber(LHV=LHV_plant)
    annotation (Placement(transformation(extent={{-450,-10},{-430,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source_fuel(h_out(
        start=0.9e6)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-440,-106})));
  Sensors_Control.FlueGases.PressureSensor                   compressor_P_out_sensor(sensor_function="Calibration", causality="compressor_tau")
    annotation (Placement(transformation(extent={{-490,-6},{-478,6}})));
  Sensors_Control.FlueGases.TemperatureSensor                   compressor_T_out_sensor(sensor_function="Calibration",
    causality="compressor_eta_is",
    T_0=723.15)
    annotation (Placement(transformation(extent={{-472,-6},{-460,6}})));
  Sensors_Control.FlueGases.PressureSensor                   turbine_P_out_sensor(sensor_function="Calibration", causality="hrsg_kf_hot")
    annotation (Placement(transformation(extent={{-350,-6},{-338,6}})));
  MetroscopeModelingLibrary.Sensors_Control.Power.PowerSensor W_GT_sensor(sensor_function="Calibration", causality="turbine_eta_is")
    annotation (Placement(transformation(extent={{-346,94},{-334,106}})));
  Sensors_Control.FlueGases.TemperatureSensor                   turbine_T_out_sensor(sensor_function="BC", T_0=913.15)
    annotation (Placement(transformation(extent={{-370,-6},{-358,6}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater Reheater(nominal_DT_default=true,
      QCp_max_side="hot")
    annotation (Placement(transformation(extent={{-108,-30},{-48,30}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPsteamTurbine annotation (Placement(transformation(extent={{-14,264},{20,296}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   T_w_ReH_out_sensor(sensor_function="Calibration", causality="RHT_Kth")
    annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=270,
        origin={-90,50})));
  Sensors_Control.WaterSteam.PressureSensor                   P_w_ReH_out_sensor(sensor_function="Calibration", causality="LPST_valve_CV")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-90,70})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_Cond_sensor(sensor_function="Calibration", causality="Cond_Kth",
    display_unit="mbar",
    signal_unit="mbar")
    annotation (Placement(transformation(extent={{28,274},{40,286}})));
  Sensors_Control.FlueGases.PressureSensor                   P_source_air_sensor(sensor_function="BC")
    annotation (Placement(transformation(extent={{-646,-6},{-634,6}})));
  Sensors_Control.FlueGases.TemperatureSensor                   T_source_air_sensor(sensor_function="BC")
    annotation (Placement(transformation(extent={{-626,-6},{-614,6}})));
  Sensors_Control.FlueGases.FlowSensor                   Q_source_air_sensor(sensor_function="BC")
    annotation (Placement(transformation(extent={{-606,-6},{-594,6}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_circulating_water_in_sensor(sensor_function="BC")
    annotation (Placement(transformation(
        extent={{5,5},{-5,-5}},
        rotation=180,
        origin={-20,200})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_circulating_water_in_sensor(sensor_function="BC")
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={0,200})));
  WaterSteam.Pipes.SlideValve                             LPST_control_valve
    annotation (Placement(transformation(extent={{-61.25,276.738},{-44.75,294.677}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_LPST_in_sensor(sensor_function="Calibration", causality="LPST_Cst")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-28,280})));
  WaterSteam.Machines.FixedSpeedPump                 pumpRec(Q_0=1)
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        origin={94,80.5455},
        rotation=0)));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_pumpRec_out_sensor(sensor_function="Calibration", causality="rh")
    annotation (Placement(transformation(
        extent={{5,5},{-5,-5}},
        rotation=180,
        origin={115,80.5455})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_pumpRec_out_sensor(sensor_function="Calibration", causality="hn")
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={131,80.5455})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.FlowSensor Q_pumpRec_out_sensor
    annotation (Placement(transformation(extent={{140,75.5455},{150,85.5455}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_w_eco_in_sensor(sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-5,5},{5,-5}},
        rotation=180,
        origin={140,34})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve pumpRec_controlValve
    annotation (Placement(transformation(extent={{153.5,77.4545},{166.5,91.4545}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor pumpRec_opening_sensor(sensor_function="Calibration", causality="Cvmax",
    output_signal_unit="")
    annotation (Placement(transformation(extent={{155,95},{165,105}})));
  MetroscopeModelingLibrary.Sensors_Control.FlueGases.PressureSensor P_flue_gas_sink_sensor(sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={222,198})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{110,310},{130,330}})));
  Sensors_Control.Fuel.PressureSensor                   P_fuel_source_sensor(sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-440,-40})));
  Sensors_Control.Fuel.TemperatureSensor                   T_fuel_source_sensor(sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-440,-64})));
  Sensors_Control.Fuel.FlowSensor                   Q_fuel_source_sensor(causality="")
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-440,-20})));
  MetroscopeModelingLibrary.Power.Machines.Generator GT_generator
    annotation (Placement(transformation(extent={{-380,90},{-348,110}})));
  MetroscopeModelingLibrary.Power.Machines.Generator ST_generator
    annotation (Placement(transformation(extent={{50,310},{82,330}})));
  MetroscopeModelingLibrary.Sensors_Control.FlueGases.TemperatureSensor T_flue_gas_sink_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={170,0})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Filter AirFilter
    annotation (Placement(transformation(extent={{-576,-10},{-556,10}})));
  Sensors_Control.FlueGases.PressureSensor                   P_filter_out_sensor(
    display_unit="mbar",                                                         sensor_function="Calibration", causality="Filter_Kfr",
    signal_unit="barA")
    annotation (Placement(transformation(extent={{-546,-6},{-534,6}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater2(QCp_max_side="hot")
    annotation (Placement(transformation(extent={{-302,-30},{-242,30}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   T_w_HPSH2_out_sensor(sensor_function="BC", display_unit="degC")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-280,60})));
  Sensors_Control.WaterSteam.PressureSensor                   P_w_HPSH2_out_sensor(sensor_function="Calibration", causality="HPST_valve_CV")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-280,80})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve deSH_controlValve
    annotation (Placement(transformation(extent={{-173.75,117.454},{-186.25,131.455}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor deSH_opening_sensor(sensor_function="Calibration", causality="Cvmax",
    output_signal_unit="")
    annotation (Placement(transformation(extent={{-185,139},{-175,149}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.FlowSensor Q_deSH_sensor(sensor_function="Calibration", causality="SH2_Kth")
    annotation (Placement(transformation(extent={{-132,114},{-144,126}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve Evap_controlValve
    annotation (Placement(transformation(extent={{36.25,31.4545},{23.75,45.455}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor Evap_opening_sensor(sensor_function="Calibration", causality="Cvmax",
    output_signal_unit="")
    annotation (Placement(transformation(extent={{25,53},{35,63}})));
  MetroscopeModelingLibrary.MultiFluid.Converters.MoistAir_to_FlueGases moistAir_to_FlueGases annotation (Placement(transformation(extent={{-682,-10},{-662,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source_air(h_out(start=47645.766)) annotation (Placement(transformation(extent={{-744,-10},{-724,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor T_HPST_out_sensor(sensor_function="Calibration", causality="HPST_eta_is")
                                                                                   annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=180,
        origin={-110,180})));
  Sensors.Displayer.WaterDisplayer displayer annotation (Placement(transformation(extent={{-248,170},{-228,190}})));
  Sensors.Displayer.FuelDisplayer fuelDisplayer annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-440,-80})));
  Sensors.Displayer.MoistAirDisplayer moistAirDisplayer annotation (Placement(transformation(extent={{-710,-10},{-690,10}})));
  Sensors.Displayer.FlueGasesDisplayer flueGasesDisplayer annotation (Placement(transformation(extent={{-664,-10},{-644,10}})));
  Sensors_Control.MoistAir.RelativeHumiditySensor Relative_Humidity_sensor(sensor_function="BC") annotation (Placement(transformation(extent={{-722,-6},{-710,6}})));
  Utilities.Interfaces.RealInput Relative_Humidity(start=0.5)
                                                   annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-716,14}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput P_source_air(start=1) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-640,14}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealExpression
                                 T_source_air(y=24 + time)
                                                        annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-620,28}),           iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput Q_source_air(start=500) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-600,14}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_filter_out(
                                              start=0.9) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-540,20}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput compressor_P_out(
                                                  start=17) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=-90,
        origin={-484,14}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealExpression
                                  Filter_Kfr(y=0.04432005 + T_source_air*0.0001)
                                             "P_filter_out"
                                             annotation (Placement(transformation(
        extent={{-17.5,-11.5},{17.5,11.5}},
        rotation=0,
        origin={-566.5,23.5}),
                          iconTransformation(extent={{-628,-4},{-608,16}})));
  Utilities.Interfaces.RealInput  compression_rate(start=18.88889)
                                                   annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-528,18}),iconTransformation(extent={{-628,-4},{-608,16}})));
  Utilities.Interfaces.RealInput  compressor_eta_is(start=0.878675)
                                                    annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-520,24}),iconTransformation(extent={{-628,-4},{-608,16}})));
  Utilities.Interfaces.RealOutput compressor_T_out(
                                                  start=450) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-466,14}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput Q_fuel_source           annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-450,-20}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput P_fuel_source(start=30) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-450,-40}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput T_fuel_source(start=156) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-450,-60}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput  turbine_eta_is(start=0.8304104)
                                                 annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-412,20}), iconTransformation(extent={{-454,4},{-434,24}})));
  Utilities.Interfaces.RealInput  turbine_T_out(
                                               start=640) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-364,12}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput turbine_P_out(
                                               start=1.1) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-344,12}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput  economizer_Kfr_cold(nominal=1e-3, start=1e-3)
                                                      annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={114,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  economizer_Kth(start=3104.9373, nominal=3104.9373)
                                                 annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={86,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  FlueGases.Pipes.FrictionPipe HRSG_friction annotation (Placement(transformation(extent={{30,10},{50,-10}})));
  Utilities.Interfaces.RealInput  HRSG_friction_Kfr(start=0.022388678) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={40,-10}), iconTransformation(extent={{-36,-16},{4,24}})));
  Utilities.Interfaces.RealInput  Reheater_Kth(start=410.44293, nominal=410.44293)
                                                                       annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-92,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  Reheater_Kfr_cold(nominal=134.2858)
                                                                  annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-62,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  HPsuperheater1_Kfr_cold(nominal=7030.31)
                                                                        annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-140,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  HPsuperheater1_Kth(start=1213.6362, nominal=1213.6362)
                                                                             annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-170,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  HPsuperheater2_Kfr_cold(nominal=2538.3271)
                                                                        annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-256,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealInput  HPsuperheater2_Kth(start=1673.8336, nominal=1673.8336)
                                                                             annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-286,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Utilities.Interfaces.RealOutput P_w_HPSH1_out(
                                               start=116) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-208,44}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_w_HPSH1_out(
                                               start=450) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-184,44}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_w_HPSH1_out1(
                                                start=9)  annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-100,70}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_w_HPSH1_out1(
                                                start=350)
                                                          annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-100,50}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_w_HPSH2_out(
                                               start=114) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-290,80}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput T_w_HPSH2_out(start=566.5) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-290,60}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_w_evap_out(
                                              start=120) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-40,44}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_w_eco_out(
                                             start=320) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={12,44}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_w_eco_out(
                                             start=122.5) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={56,44}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput T_w_eco_in(start=85) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={140,44}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_pumpRec_out(
                                               start=324) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={114,92}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_pumpRec_out(
                                               start=180) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={130,92}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput P_flue_gas_sink(start=1) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={210,198}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput W_GT(
                                      start=150) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-340,112}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_pump_out(
                                            start=170) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={150,170}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_pump_out(
                                            start=35) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={130,170}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput Q_pump_out(
                                            start=50) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={170,170}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_circulating_water_out(
                                                         start=25) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={110,210}),iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput T_circulating_water_in(start=15) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-20,210}),
                         iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput P_circulating_water_in(start=5) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={0,210}),  iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_Cond(
                                        start=50) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={34,292}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_LPST_in(
                                           start=8) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-28,292}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_HPST_out(
                                            start=10) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-128,190}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput T_HPST_out(
                                            start=255.5) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-110,190}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput Q_deSH(
                                        start=2) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-138,132}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput  pumpRec_hn(start=870.66187)
                                             annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={89,107}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  pumpRec_rh(start=0.6881236)
                                             annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={81,97}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  pump_hn(start=1735.4259)
                                          annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={104,180}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  pump_rh(start=0.70563865)
                                          annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={96,170}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  LPsteamTurbine_Cst(start=411424.22)
                                                     annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-13,301}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  LPsteamTurbine_eta_is(start=0.8438316)
                                                        annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-9,309}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  HPsteamTurbine_Cst(start=6.038082e+07)
                                                     annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-179,201}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  HPsteamTurbine_eta_is(start=0.8438316)
                                                        annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-165,211}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  LPST_control_valve_Cv(start=69310.586)
                                                        annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-67,289}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  HPST_control_valve_Cv(start=6647.2905)
                                                        annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=0,
        origin={-227,189}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealOutput T_flue_gas_sink(start=1) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={170,10}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput W_ST_out(
                                          start=65) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={96,332}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput P_HPST_in(
                                           start=113) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-194,192}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput deSH_opening(
                                              start=0.15)
                                                         annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={-180,154}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput Evap_opening(
                                              start=0.35) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={30,68}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealOutput pumpRec_opening(
                                                 start=0.35) annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=270,
        origin={160,110}), iconTransformation(extent={{-754,-34},{-714,6}})));
  Utilities.Interfaces.RealInput  deSH_controlValve_Cv_max(start=7.7502966)
                                                           annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=180,
        origin={-167,127}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  Evap_controlValve_Cv_max(start=539.1173)
                                                           annotation (Placement(transformation(
        extent={{-3,-3},{3,3}},
        rotation=180,
        origin={39,41}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  pumpRec_controlValve_Cv_max(start=52.329174)
                                                              annotation (Placement(transformation(
        extent={{3,-3},{-3,3}},
        rotation=180,
        origin={147,105}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealOutput Q_pumpRec_out annotation (Placement(transformation(
        extent={{3,-3},{-3,3}},
        rotation=90,
        origin={145,93}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  condenser_Kth(start=93661.23)
                                                annotation (Placement(transformation(
        extent={{3,-3},{-3,3}},
        rotation=90,
        origin={40,230}), iconTransformation(extent={{46,68},{66,88}})));
  Utilities.Interfaces.RealInput  condenser_Qv_cold_in(start=2.7349906)
                                                       annotation (Placement(transformation(
        extent={{3,-3},{-3,3}},
        rotation=90,
        origin={20,230}), iconTransformation(extent={{46,68},{66,88}})));
  Modelica.Blocks.Sources.RealExpression condenser_Kfr_cold(y=0) annotation (Placement(transformation(
        extent={{-3,-8},{3,8}},
        rotation=270,
        origin={32,225})));
  Modelica.Blocks.Sources.RealExpression combustionChamber_eta(y=0.9999) annotation (Placement(transformation(
        extent={{-4,-8},{4,8}},
        rotation=270,
        origin={-448,24})));
  Modelica.Blocks.Sources.RealExpression combustionChamber_Kfr(y=1e-3)
                                                                    annotation (Placement(transformation(
        extent={{-4,-8},{4,8}},
        rotation=270,
        origin={-432,24})));
  Utilities.Interfaces.RealInput  Evap_Kth(nominal=3383.7917)
                                                         annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-8,-30}), iconTransformation(extent={{-62,-16},{-42,4}})));
  Sensors_Control.Fuel.Chromatograph chromatograph annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-441,-91})));
  Utilities.Interfaces.BoundaryCondition X_CO2(start=2.5)   annotation (Placement(transformation(extent={{-452,-80},{-444,-72}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
  Utilities.Interfaces.BoundaryCondition X_N2(start=2.5)   annotation (Placement(transformation(extent={{-478,-86},{-470,-78}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
  Utilities.Interfaces.BoundaryCondition X_C4H10(start=0) annotation (Placement(transformation(extent={{-472,-104},{-464,-96}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
  Utilities.Interfaces.BoundaryCondition X_C3H8(start=0) annotation (Placement(transformation(extent={{-464,-116},{-456,-108}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
  Utilities.Interfaces.BoundaryCondition X_C2H6(start=05)   annotation (Placement(transformation(extent={{-436,-118},{-428,-110}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
  Utilities.Interfaces.BoundaryCondition X_CH4(start=90)  annotation (Placement(transformation(extent={{-418,-120},{-410,-112}}), iconTransformation(extent={{-472,-112},{-432,-72}})));
equation



  //--- Air / Flue Gas System ---
    // Fuel Source
      //  Quantities definition
      //source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Condenser
      // Parameters
      condenser.C_incond = 0;

  connect(HPsuperheater1.C_cold_out, T_w_HPSH1_out_sensor.C_in) annotation (
     Line(points={{-168,24},{-166,24},{-166,34},{-178,34}},
                                                          color={28,108,200}));
  connect(P_HPST_out_sensor.C_in, HPsteamTurbine.C_out)
    annotation (Line(points={{-134,180},{-146,180}}, color={28,108,200}));
  connect(P_HPST_in_sensor.C_out, HPsteamTurbine.C_in)
    annotation (Line(points={{-188,180},{-180,180}}, color={28,108,200}));
  connect(T_circulating_water_out_sensor.C_out, circulating_water_sink.C_in)
    annotation (Line(points={{115,200},{126,200}},
                                                 color={28,108,200}));
  connect(combustionChamber.outlet,gasTurbine. C_in) annotation (Line(points={{-430,0},{-412,0}},
                                                                                              color={95,95,95}));
  connect(airCompressor.C_out,compressor_P_out_sensor. C_in) annotation (Line(points={{-496,0},{-490,0}},
                                                                                                        color={95,95,95}));
  connect(compressor_P_out_sensor.C_out,compressor_T_out_sensor. C_in) annotation (Line(points={{-478,0},{-472,0}},
                                                                                                                  color={95,95,95}));
  connect(compressor_T_out_sensor.C_out,combustionChamber. inlet) annotation (Line(points={{-460,0},{-450,0}},
                                                                                                             color={95,95,95}));
  connect(evaporator.C_cold_in, T_w_eco_out_sensor.C_out) annotation (Line(
        points={{3.2,24},{3.2,34},{6,34}},                     color={28,108,200}));
  connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in)
    annotation (Line(points={{79.6,200},{92,200},{92,200},{105,200}},
                    color={28,108,200}));
  connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{60,185.778},{60,160},{103,160}},
                              color={28,108,200}));
  connect(HPsuperheater1.C_cold_in, P_w_evap_out_sensor.C_out) annotation (Line(
        points={{-144,24},{-144,34},{-46,34}},color={28,108,200}));
  connect(evaporator.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
        points={{-19.2,24},{-20,24},{-20,34},{-34,34}}, color={28,108,200}));
  connect(economiser.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
        points={{88.4,23.6},{88,23.6},{88,34},{62,34}},
                                                   color={28,108,200}));
  connect(gasTurbine.C_out, turbine_T_out_sensor.C_in)
    annotation (Line(points={{-380,0},{-370,0}},     color={95,95,95}));
  connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out)
    annotation (Line(points={{-350,0},{-358,0}},     color={95,95,95}));
  connect(evaporator.C_hot_in, Reheater.C_hot_out) annotation (Line(points={{-36,0},{-48,0}},
                                           color={95,95,95}));
  connect(HPsuperheater1.C_hot_out, Reheater.C_hot_in)
    annotation (Line(points={{-126,0},{-108,0}},    color={95,95,95}));
  connect(Reheater.C_cold_out, T_w_ReH_out_sensor.C_in)
    annotation (Line(points={{-90,24},{-90,44}},          color={28,108,200}));
  connect(P_Cond_sensor.C_in, LPsteamTurbine.C_out)
    annotation (Line(points={{28,280},{20,280}},   color={28,108,200}));
  connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,280},{60,280},{60,218.134}},
                                     color={28,108,200}));

  connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
    annotation (Line(points={{-634,0},{-626,0}},     color={95,95,95}));
  connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
    annotation (Line(points={{-614,0},{-606,0}},     color={95,95,95}));
  connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
     Line(points={{40,200},{22,200},{22,200},{5,200}},                color={28,
          108,200}));
  connect(P_LPST_in_sensor.C_out, LPsteamTurbine.C_in)
    annotation (Line(points={{-22,280},{-14,280}},  color={28,108,200}));
  connect(P_LPST_in_sensor.C_in, LPST_control_valve.C_out) annotation (Line(
        points={{-34,280},{-40,280},{-40,280},{-44.75,280}},             color={
          28,108,200}));
  connect(P_w_ReH_out_sensor.C_out, LPST_control_valve.C_in) annotation (Line(
        points={{-90,76},{-90,280},{-61.25,280}}, color={28,108,200}));
  connect(P_w_ReH_out_sensor.C_in, T_w_ReH_out_sensor.C_out)
    annotation (Line(points={{-90,64},{-90,56}}, color={28,108,200}));
  connect(T_w_HPSH1_out_sensor.C_out, P_w_HPSH1_out_sensor.C_in)
    annotation (Line(points={{-190,34},{-202,34}},
                                                 color={28,108,200}));
  connect(HPST_control_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
        points={{-206.75,180},{-204,180},{-204,180},{-200,180}}, color={28,108,200}));
  connect(T_circulating_water_in_sensor.C_out, P_circulating_water_in_sensor.C_in)
    annotation (Line(points={{-15,200},{-5,200}},  color={28,108,200}));
  connect(circulating_water_source.C_out, T_circulating_water_in_sensor.C_in)
    annotation (Line(points={{-35,200},{-25,200}},           color={28,108,200}));
  connect(pump.C_out, T_pump_out_sensor.C_in)
    annotation (Line(points={{117,160},{125,160}},     color={28,108,200}));
  connect(T_pump_out_sensor.C_out, P_pump_out_sensor.C_in) annotation (Line(
        points={{135,160},{145,160}},             color={28,108,200}));
  connect(P_pump_out_sensor.C_out, Q_pump_out_sensor.C_in) annotation (Line(
        points={{155,160},{165,160}},             color={28,108,200}));
  connect(P_pumpRec_out_sensor.C_out, Q_pumpRec_out_sensor.C_in)
    annotation (Line(points={{136,80.5455},{140,80.5455}},
                                               color={28,108,200}));
  connect(pumpRec_controlValve.Opening, pumpRec_opening_sensor.Opening)
    annotation (Line(points={{160,90.1818},{160,94.9}},         color={0,0,127}));
  connect(pumpRec.C_in, P_w_eco_out_sensor.C_in) annotation (Line(points={{87,80.5455},{80,80.5455},{80,34},{62,34}},
                                   color={28,108,200}));
  connect(economiser.C_cold_in, T_w_eco_in_sensor.C_out) annotation (Line(
        points={{111.6,23.6},{111.6,34},{135,34}},
                                              color={28,108,200}));
  connect(T_w_eco_in_sensor.C_in, loopBreaker.C_out) annotation (Line(points={{145,34},{180,34},{180,44}},
                                       color={28,108,200}));
  connect(Q_pump_out_sensor.C_out, loopBreaker.C_in)
    annotation (Line(points={{175,160},{180,160},{180,64}},
                                                         color={28,108,200}));
  connect(T_pumpRec_out_sensor.C_out, P_pumpRec_out_sensor.C_in) annotation (
      Line(points={{120,80.5455},{126,80.5455}},                         color={
          28,108,200}));
  connect(Q_pumpRec_out_sensor.C_out, pumpRec_controlValve.C_in)
    annotation (Line(points={{150,80.5455},{154,80.5455},{154,80},{153.5,80}},
                                                         color={28,108,200}));
  connect(flue_gas_sink.C_in, P_flue_gas_sink_sensor.C_out)
    annotation (Line(points={{222,219},{222,204}}, color={95,95,95}));
  connect(sink.C_in, W_ST_out_sensor.C_out)
    annotation (Line(points={{115,320},{101.88,320}},color={244,125,35}));
  connect(combustionChamber.inlet1, Q_fuel_source_sensor.C_out)
    annotation (Line(points={{-440,-10},{-440,-15}}, color={213,213,0}));
  connect(Q_fuel_source_sensor.C_in, P_fuel_source_sensor.C_out) annotation (
      Line(points={{-440,-25},{-440,-35}},                       color={213,213,
          0}));
  connect(P_fuel_source_sensor.C_in, T_fuel_source_sensor.C_out) annotation (
      Line(points={{-440,-45},{-440,-59}},                       color={213,213,
          0}));
  connect(GT_generator.C_out, W_GT_sensor.C_in)
    annotation (Line(points={{-352.8,100},{-346,100}},
                                                     color={244,125,35}));
  connect(W_GT_sensor.C_out, sink_power.C_in)
    annotation (Line(points={{-334.12,100},{-327,100}},
                                                      color={244,125,35}));
  connect(GT_generator.C_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-373.92,100},{-380,100},{-380,16}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(W_ST_out_sensor.C_in, ST_generator.C_out)
    annotation (Line(points={{90,320},{77.2,320}},color={244,125,35}));
  connect(HPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{-146,193.44},{-146,193.44},{-146,266},{-146,320},{56.08,320}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{20,293.44},{20,320},{56.08,320}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(pumpRec_controlValve.C_out, loopBreaker.C_in) annotation (Line(points={{166.5,80},{180,80},{180,64}},
                                                                color={28,108,200}));
  connect(pumpRec.C_out, T_pumpRec_out_sensor.C_in) annotation (Line(points={{101,80.5455},{110,80.5455}},
                                               color={28,108,200}));
  connect(AirFilter.C_out, P_filter_out_sensor.C_in)
    annotation (Line(points={{-556,0},{-546,0}},     color={95,95,95}));
  connect(Q_source_air_sensor.C_out, AirFilter.C_in)
    annotation (Line(points={{-594,0},{-576,0}},     color={95,95,95}));
  connect(P_filter_out_sensor.C_out, airCompressor.C_in)
    annotation (Line(points={{-534,0},{-524,0}},     color={95,95,95}));
  connect(HPsuperheater1.C_hot_in, HPsuperheater2.C_hot_out)
    annotation (Line(points={{-186,0},{-242,0}},     color={95,95,95}));
  connect(P_w_HPSH1_out_sensor.C_out, HPsuperheater2.C_cold_in) annotation (
     Line(points={{-214,34},{-260,34},{-260,24}},
                                                color={28,108,200}));
  connect(turbine_P_out_sensor.C_out, HPsuperheater2.C_hot_in)
    annotation (Line(points={{-338,0},{-302,0}},     color={95,95,95}));
  connect(deSH_opening_sensor.Opening, deSH_controlValve.Opening)
    annotation (Line(points={{-180,138.9},{-180,130.182}}, color={0,0,127}));
  connect(Q_deSH_sensor.C_in, loopBreaker.C_in) annotation (Line(points={{-132,120},{180,120},{180,64}},
                              color={28,108,200}));
  connect(Q_deSH_sensor.C_out, deSH_controlValve.C_in) annotation (Line(points={{-144,120},{-158,120},{-158,120},{-173.75,120}},
                                                       color={28,108,200}));
  connect(deSH_controlValve.C_out, HPsuperheater2.C_cold_in) annotation (Line(
        points={{-186.25,120},{-230,120},{-230,34},{-260,34},{-260,24}},
                                                                     color={28,108,
          200}));
  connect(T_w_HPSH2_out_sensor.C_in, HPsuperheater2.C_cold_out) annotation (
      Line(points={{-280,54},{-280,24},{-284,24}},           color={28,108,200}));
  connect(P_w_HPSH2_out_sensor.C_in, T_w_HPSH2_out_sensor.C_out)
    annotation (Line(points={{-280,74},{-280,66}}, color={28,108,200}));
  connect(P_w_eco_out_sensor.C_out, Evap_controlValve.C_in) annotation (Line(
        points={{50,34},{44,34},{44,34},{36.25,34}},                 color={28,108,
          200}));
  connect(T_w_eco_out_sensor.C_in, Evap_controlValve.C_out) annotation (Line(
        points={{18,34},{20,34},{20,34},{23.75,34}},                 color={28,108,
          200}));
  connect(Evap_controlValve.Opening, Evap_opening_sensor.Opening)
    annotation (Line(points={{30,44.1822},{30,52.9}}, color={0,0,127}));
  connect(economiser.C_hot_out, T_flue_gas_sink_sensor.C_in) annotation (Line(
        points={{129,0},{164,0}},                     color={95,95,95}));
  connect(P_HPST_out_sensor.C_out, T_HPST_out_sensor.C_in) annotation (Line(points={{-122,180},{-116,180}},color={28,108,200}));
  connect(T_HPST_out_sensor.C_out, Reheater.C_cold_in) annotation (Line(points={{-104,180},{-66,180},{-66,24}},color={28,108,200}));
  connect(airCompressor.C_W_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-496,10.5},{-496,34},{-380,34},{-380,16}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(T_flue_gas_sink_sensor.C_out, P_flue_gas_sink_sensor.C_in) annotation (Line(points={{176,0},{222,0},{222,192}},                         color={95,95,95}));
  connect(HPST_control_valve.C_in, displayer.C_out) annotation (Line(points={{-223.25,180},{-230,180},{-230,180},{-235,180}}, color={28,108,200}));
  connect(displayer.C_in, P_w_HPSH2_out_sensor.C_out) annotation (Line(points={{-241,180},{-280,180},{-280,86}},                     color={28,108,200}));
  connect(fuelDisplayer.C_out, T_fuel_source_sensor.C_in) annotation (Line(points={{-440,-77},{-440,-69}}, color={213,213,0}));
  connect(moistAir_to_FlueGases.inlet, moistAirDisplayer.C_out) annotation (Line(points={{-682,0},{-697,0}},     color={85,170,255}));
  connect(P_source_air_sensor.C_in, flueGasesDisplayer.C_out) annotation (Line(points={{-646,0},{-651,0}},     color={95,95,95}));
  connect(flueGasesDisplayer.C_in, moistAir_to_FlueGases.outlet) annotation (Line(points={{-657,0},{-662,0}},     color={95,95,95}));
  connect(moistAirDisplayer.C_in, Relative_Humidity_sensor.C_out) annotation (Line(points={{-703,0},{-710,0}},     color={85,170,255}));
  connect(Relative_Humidity_sensor.C_in, source_air.C_out) annotation (Line(points={{-722,0},{-729,0}},     color={85,170,255}));
  connect(Relative_Humidity_sensor.H_sensor, Relative_Humidity) annotation (Line(points={{-716,6},{-716,14}},   color={0,0,127}));
  connect(P_source_air_sensor.P_sensor, P_source_air) annotation (Line(points={{-640,6},{-640,14}},  color={0,0,127}));
  connect(Q_source_air_sensor.Q_sensor, Q_source_air) annotation (Line(points={{-600,6},{-600,14}},  color={0,0,127}));
  connect(P_filter_out_sensor.P_sensor, P_filter_out) annotation (Line(points={{-540,6},{-540,20}},  color={0,0,127}));
  connect(compressor_P_out_sensor.P_sensor, compressor_P_out) annotation (Line(points={{-484,6},{-484,14}},  color={0,0,127}));
  connect(compression_rate, airCompressor.tau) annotation (Line(points={{-528,18},{-528,12.25},{-522.6,12.25}},  color={0,0,127}));
  connect(compressor_T_out, compressor_T_out_sensor.T_sensor) annotation (Line(points={{-466,14},{-466,6}},  color={0,0,127}));
  connect(Q_fuel_source_sensor.Q_sensor, Q_fuel_source) annotation (Line(points={{-445,-20},{-450,-20},{-450,-20},{-450,-20}},
                                                                                                         color={0,0,127}));
  connect(P_fuel_source_sensor.P_sensor, P_fuel_source) annotation (Line(points={{-445,-40},{-450,-40}}, color={0,0,127}));
  connect(T_fuel_source_sensor.T_sensor, T_fuel_source) annotation (Line(points={{-445,-64},{-448,-64},{-448,-60},{-450,-60}},
                                                                                                         color={0,0,127}));
  connect(gasTurbine.eta_is, turbine_eta_is) annotation (Line(points={{-412,12.8},{-412,18},{-412,20},{-412,20}},
                                                                                              color={0,0,127}));
  connect(turbine_T_out_sensor.T_sensor, turbine_T_out) annotation (Line(points={{-364,6},{-364,12}}, color={0,0,127}));
  connect(turbine_P_out_sensor.P_sensor, turbine_P_out) annotation (Line(points={{-344,6},{-344,12}}, color={0,0,127}));
  connect(economiser.Kfr_cold, economizer_Kfr_cold) annotation (Line(points={{114.5,-20.65},{114.5,-22.325},{114,-22.325},{114,-30}}, color={0,0,127}));
  connect(economiser.Kth, economizer_Kth) annotation (Line(points={{85.5,-20.65},{86,-20.65},{86,-30}}, color={0,0,127}));
  connect(evaporator.C_hot_out,HRSG_friction. C_in) annotation (Line(points={{20,0},{30,0}},                color={95,95,95}));
  connect(HRSG_friction.C_out, economiser.C_hot_in) annotation (Line(points={{50,0},{71,0}},            color={95,95,95}));
  connect(HRSG_friction.Kfr, HRSG_friction_Kfr) annotation (Line(points={{40,-4},{40,-10},{40,-10},{40,-10}},
                                                                                            color={0,0,127}));
  connect(Reheater_Kfr_cold, Reheater.Kfr_cold) annotation (Line(points={{-62,-30},{-63,-30},{-63,-21}}, color={0,0,127}));
  connect(Reheater_Kth, Reheater.Kth) annotation (Line(points={{-92,-30},{-93,-30},{-93,-21}}, color={0,0,127}));
  connect(HPsuperheater1.Kfr_cold, HPsuperheater1_Kfr_cold) annotation (Line(points={{-141,-21},{-140,-21},{-140,-30}}, color={0,0,127}));
  connect(HPsuperheater1.Kth, HPsuperheater1_Kth) annotation (Line(points={{-171,-21},{-170,-21},{-170,-30}}, color={0,0,127}));
  connect(HPsuperheater2.Kth, HPsuperheater2_Kth) annotation (Line(points={{-287,-21},{-286,-21},{-286,-30}}, color={0,0,127}));
  connect(HPsuperheater2.Kfr_cold, HPsuperheater2_Kfr_cold) annotation (Line(points={{-257,-21},{-256,-21},{-256,-30}}, color={0,0,127}));
  connect(T_w_HPSH1_out_sensor.T_sensor, T_w_HPSH1_out) annotation (Line(points={{-184,40},{-184,44}}, color={0,0,127}));
  connect(P_w_HPSH1_out_sensor.P_sensor, P_w_HPSH1_out) annotation (Line(points={{-208,40},{-208,44}}, color={0,0,127}));
  connect(T_w_HPSH1_out1, T_w_ReH_out_sensor.T_sensor) annotation (Line(points={{-100,50},{-96,50}},           color={0,0,127}));
  connect(P_w_ReH_out_sensor.P_sensor, P_w_HPSH1_out1) annotation (Line(points={{-96,70},{-100,70}},          color={0,0,127}));
  connect(T_w_HPSH2_out_sensor.T_sensor, T_w_HPSH2_out) annotation (Line(points={{-286,60},{-290,60}}, color={0,0,127}));
  connect(P_w_HPSH2_out, P_w_HPSH2_out_sensor.P_sensor) annotation (Line(points={{-290,80},{-286,80}}, color={0,0,127}));
  connect(P_w_evap_out, P_w_evap_out_sensor.P_sensor) annotation (Line(points={{-40,44},{-40,40}}, color={0,0,127}));
  connect(T_w_eco_out, T_w_eco_out_sensor.T_sensor) annotation (Line(points={{12,44},{12,40}},
                                                                                             color={0,0,127}));
  connect(P_w_eco_out_sensor.P_sensor, P_w_eco_out) annotation (Line(points={{56,40},{56,44}}, color={0,0,127}));
  connect(T_w_eco_in, T_w_eco_in_sensor.T_sensor) annotation (Line(points={{140,44},{140,39}}, color={0,0,127}));
  connect(T_pumpRec_out, T_pumpRec_out_sensor.T_sensor) annotation (Line(points={{114,92},{115,92},{115,85.5455}}, color={0,0,127}));
  connect(P_pumpRec_out_sensor.P_sensor, P_pumpRec_out) annotation (Line(points={{131,85.5455},{131,86.773},{130,86.773},{130,92}}, color={0,0,127}));
  connect(P_flue_gas_sink_sensor.P_sensor, P_flue_gas_sink) annotation (Line(points={{216,198},{210,198}}, color={0,0,127}));
  connect(W_GT, W_GT_sensor.W_sensor) annotation (Line(points={{-340,112},{-340,106}}, color={0,0,127}));
  connect(T_pump_out, T_pump_out_sensor.T_sensor) annotation (Line(points={{130,170},{130,165}},           color={0,0,127}));
  connect(P_pump_out, P_pump_out_sensor.P_sensor) annotation (Line(points={{150,170},{150,165}},           color={0,0,127}));
  connect(Q_pump_out_sensor.Q_sensor, Q_pump_out) annotation (Line(points={{170,165},{170,170}},           color={0,0,127}));
  connect(T_circulating_water_out_sensor.T_sensor, T_circulating_water_out) annotation (Line(points={{110,205},{110,210}},                     color={0,0,127}));
  connect(P_circulating_water_in_sensor.P_sensor, P_circulating_water_in) annotation (Line(points={{0,205},{0,210}},                     color={0,0,127}));
  connect(T_circulating_water_in_sensor.T_sensor, T_circulating_water_in) annotation (Line(points={{-20,205},{-20,210}},             color={0,0,127}));
  connect(P_Cond_sensor.P_sensor, P_Cond) annotation (Line(points={{34,286},{34,292}}, color={0,0,127}));
  connect(P_LPST_in_sensor.P_sensor, P_LPST_in) annotation (Line(points={{-28,286},{-28,292}}, color={0,0,127}));
  connect(P_HPST_out, P_HPST_out_sensor.P_sensor) annotation (Line(points={{-128,190},{-128,186}}, color={0,0,127}));
  connect(T_HPST_out_sensor.T_sensor, T_HPST_out) annotation (Line(points={{-110,186},{-110,190}}, color={0,0,127}));
  connect(Q_deSH_sensor.Q_sensor, Q_deSH) annotation (Line(points={{-138,126},{-138,132}}, color={0,0,127}));
  connect(pumpRec_hn, pumpRec.hn) annotation (Line(points={{89,107},{88.54,107},
          {88.54,86.1455}},                                                                       color={0,0,127}));
  connect(pumpRec_rh, pumpRec.rh) annotation (Line(points={{81,97},{81,83.3455},
          {87,83.3455}},                                                                          color={0,0,127}));
  connect(pump_rh, pump.rh) annotation (Line(points={{96,170},{96,162.8},{103,162.8}},    color={0,0,127}));
  connect(pump_hn, pump.hn) annotation (Line(points={{104,180},{104,165.6},{104.54,
          165.6}},                                                                          color={0,0,127}));
  connect(LPsteamTurbine_eta_is, LPsteamTurbine.eta_is) annotation (Line(points={{-9,309},
          {-0.74,309},{-0.74,294.72}},                                                                               color={0,0,127}));
  connect(LPsteamTurbine.Cst, LPsteamTurbine_Cst) annotation (Line(points={{-8.22,
          293.12},{-13,293.12},{-13,301}},                                                                       color={0,0,127}));
  connect(HPsteamTurbine_Cst, HPsteamTurbine.Cst) annotation (Line(points={{-179,
          201},{-174.22,201},{-174.22,193.12}},                                                                     color={0,0,127}));
  connect(HPsteamTurbine.eta_is, HPsteamTurbine_eta_is) annotation (Line(points={{-166.74,194.72},{-165,196},{-165,211}},   color={0,0,127}));
  connect(LPST_control_valve.Cv, LPST_control_valve_Cv) annotation (Line(points={{-56.3,288.969},{-56.3,289},{-67,289}}, color={0,0,127}));
  connect(HPST_control_valve.Cv, HPST_control_valve_Cv) annotation (Line(points={{-218.3,188.969},{-220,189},{-227,189}}, color={0,0,127}));
  connect(T_flue_gas_sink_sensor.T_sensor, T_flue_gas_sink) annotation (Line(points={{170,6},{170,10}}, color={0,0,127}));
  connect(W_ST_out_sensor.W_sensor, W_ST_out) annotation (Line(points={{96,326},{96,332}}, color={0,0,127}));
  connect(P_HPST_in_sensor.P_sensor, P_HPST_in) annotation (Line(points={{-194,186},{-194,192}}, color={0,0,127}));
  connect(deSH_opening, deSH_opening_sensor.opening_sensor) annotation (Line(points={{-180,154},{-180,149.1}}, color={0,0,127}));
  connect(Evap_opening_sensor.opening_sensor,Evap_opening)  annotation (Line(points={{30,63.1},{30,68}}, color={0,0,127}));
  connect(pumpRec_opening, pumpRec_opening_sensor.opening_sensor) annotation (Line(points={{160,110},{160,105.1}}, color={0,0,127}));
  connect(deSH_controlValve_Cv_max, deSH_controlValve.Cv_max) annotation (Line(points={{-167,127},{-172,127},{-172,127},{-177.5,127}}, color={0,0,127}));
  connect(Evap_controlValve.Cv_max, Evap_controlValve_Cv_max) annotation (Line(points={{32.5,41.0003},{34,41.0003},{34,41},{39,41}}, color={0,0,127}));
  connect(pumpRec_controlValve.Cv_max, pumpRec_controlValve_Cv_max) annotation (Line(points={{157.4,87},{152,87},{152,105},{147,105}}, color={0,0,127}));
  connect(Q_pumpRec_out_sensor.Q_sensor, Q_pumpRec_out) annotation (Line(points={{145,85.5455},{145,93}}, color={0,0,127}));
  connect(condenser_Kth, condenser.Kth) annotation (Line(points={{40,230},{40,219.556},{47.2,219.556}},
                                                                                                  color={0,0,127}));
  connect(condenser.Qv_cold_in, condenser_Qv_cold_in) annotation (Line(points={{38,214.222},{27,214.222},{27,230},{20,230}},     color={0,0,127}));
  connect(condenser.Kfr_cold, condenser_Kfr_cold.y) annotation (Line(points={{38,207.111},{32,207.111},{32,221.7}}, color={0,0,127}));
  connect(combustionChamber_Kfr.y, combustionChamber.Kfr) annotation (Line(points={{-432,19.6},{-432,16},{-437,16},{-437,11}}, color={0,0,127}));
  connect(combustionChamber_eta.y, combustionChamber.eta) annotation (Line(points={{-448,19.6},{-448,16},{-443,16},{-443,11}}, color={0,0,127}));
  connect(evaporator.Kth, Evap_Kth) annotation (Line(points={{-8,-21},{-8,-30}}, color={0,0,127}));
  connect(Filter_Kfr.y, AirFilter.Kfr) annotation (Line(points={{-566.5,17.75},{-566.5,12},{-566,12},{-566,4}}, color={0,0,127}));
  connect(airCompressor.eta_is, compressor_eta_is) annotation (Line(points={{-519.8,11.55},{-519.8,24},{-520,24}}, color={0,0,127}));
  connect(T_source_air.y, T_source_air_sensor.T_sensor) annotation (Line(points={{-620,26.5},{-620,6}}, color={0,0,127}));
  connect(source_fuel.C_out, chromatograph.C_in) annotation (Line(points={{-440,-101},{-441,-100},{-441,-96}}, color={213,213,0}));
  connect(chromatograph.C_out, fuelDisplayer.C_in) annotation (Line(points={{-441,-86},{-440,-86},{-440,-83}}, color={213,213,0}));
  connect(chromatograph.X_CO2, X_CO2) annotation (Line(points={{-442.8,-86.8},{-442.8,-88},{-448,-88},{-448,-76}},
                                                                                                        color={0,0,127}));
  connect(chromatograph.X_N2, X_N2) annotation (Line(points={{-444.9,-88},{-460,-88},{-460,-82},{-474,-82}},
                                                                                       color={0,0,127}));
  connect(chromatograph.X_C4H10_n_butane, X_C4H10) annotation (Line(points={{-446.2,-90},{-456,-90},{-456,-100},{-468,-100}},              color={0,0,127}));
  connect(chromatograph.X_C3H8, X_C3H8) annotation (Line(points={{-446.2,-92.1},{-446.2,-112},{-460,-112}}, color={0,0,127}));
  connect(chromatograph.X_C2H6, X_C2H6) annotation (Line(points={{-444.8,-94},{-444.8,-100},{-432,-100},{-432,-114}}, color={0,0,127}));
  connect(chromatograph.X_CH4, X_CH4) annotation (Line(points={{-442.9,-95.2},{-414,-95.2},{-414,-116}},  color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-760,-120},{260,440}})),
                                                              Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-760,-120},{260,440}}),
        graphics={Rectangle(
          extent={{-324,44},{246,-46}},
          pattern=LinePattern.None,
          lineColor={0,0,0},
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid,
          radius=0),                      Text(
          extent={{-160,-92},{42,-104}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="Heat Recovery Steam Generator"),
        Polygon(
          points={{-380,16},{-380,16},{-380,-16},{-324,-46},{-324,44},{-380,16}},
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{246,42},{200,216}},
          pattern=LinePattern.None,
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-230,124},{-174,124}},
          textColor={28,108,200},
          textString="Desuperheater"),    Text(
          extent={{-310,-66},{-232,-72}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Superheater 2"),    Text(
          extent={{-194,-66},{-116,-74}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Superheater 1"),    Text(
          extent={{-104,-66},{-42,-72}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Reheater"),         Text(
          extent={{-38,-66},{24,-72}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Evaporator"),       Text(
          extent={{74,-66},{136,-72}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=5,
          textString="Economizer")}));
end MetroscopiaCCGT_direct_mml4;
