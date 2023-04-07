within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_causality_reverse
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions

    // Air source
    input Real P_source_air(start=1) "bar";
    input Units.MassFlowRate Q_source_air(start=500) "kg/s";
    input Real T_source_air(start=24) "degC";
    input Real Relative_Humidity(start=0.5);
    // Fuel source
    input Real P_fuel_source(start=30) "bar";
    input Real T_fuel_source(start=156) "degC";
    input Units.SpecificEnthalpy LHV_plant(start=48130e3) "Directly assigned in combustion chamber modifiers";
    // Circulating water circuit
    input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
    input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
    // Flue gas sink
    input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";

  // Parameters

    // Gas Turbine
    parameter Units.SpecificEnthalpy GT_h_out=1e6;  // This enthalpy corresponds to T = 640°C at 1.1 bar
    parameter Real combustionChamber_eta = 0.9999;
    parameter Units.FrictionCoefficient combustionChamber_Kfr = 1e-3;
    // Economizer
    parameter String Eco_QCp_max_side = "hot";
    parameter Real T_w_eco_in = 85 "degC"; // Controlled by the economizer recirculation pump flow rate
    // Evaporator
    parameter Real Evap_x_steam_out=1;
    parameter Units.FrictionCoefficient Evap_Kfr_cold=0;
    // High Pressure Superheater
    parameter String HPSH_QCp_max_side = "hot";
    // High Pressure Superheater 2
    parameter Real T_w_HPSH2_out = 566.5 "degC"; // Controlled by the desuperheater mass flow rate
    // Low Pressure Superheater (resuperheater)
    parameter String ReH_QCp_max_side = "hot";

  // Observables used for calibration

    // Gas Turbine
    input Real P_filter_out(start=0.9) "barA";
    input Real compressor_P_out(start = 17) "barA";
    input Real compressor_T_out(start = 450) "degC";
    input Real W_GT(start = 150) "MW";
    input Real turbine_P_out(start=1.1) "barA";
    // Economizer
    input Real P_w_eco_out(start = 122.5, min= 0, nominal = 122.5) "barA";
    input Real T_w_eco_out(start = 320, min = 0, nominal = 320) "degC";
    // Evaporator
    input Real Evap_opening(start=0.35);
    input Real P_w_evap_out(start = 120, min= 0, nominal = 120) "barA";
    // High Pressure Superheater 1
    input Real P_w_HPSH1_out(start=116, min=0, nominal=116) "barA";
    input Real T_w_HPSH1_out(start=450, min= 200, nominal=450) "degC";
    // High Pressure Superheater 2
    input Real P_w_HPSH2_out(start=114, min=0, nominal=114) "barA";
    // De-superheater
    input Real deSH_opening(start=0.15);
    input Real Q_deSH(start=2) "kg/s";
    // Reheater
    input Real T_w_ReH_out(start = 350, min = 0, nominal = 350) "degC";
    input Real P_w_ReH_out(start=9, min=0, nominal=9) "barA";
    // Steam Turbine
    input Real P_ST_in(start=113) "barA";
    input Real P_ST_out(start=10) "barA";
    input Real W_ST_out(start=65, unit="MW", nominal=65, min=0) "MW";
    input Real T_HPST_out(start=255.5) "degC";
    input Real P_LPST_in(start=8, min=0, nominal=4.9) "bar";
    // Condenser
    input Real P_Cond(start=0.05, min=0, nominal=0.05) "bar";
    input Real T_circulating_water_out(start=25, min=10, nominal=25) "degC";
    // Extraction Pump
    input Real P_pump_out(start=170) "barA";
    input Real T_pump_out(start=35) "degC";
    input Real Q_pump_out(start=50) "kg/s";
    // Recirculation Pump
    input Real P_pumpRec_out(start=180) "barA";
    input Real T_pumpRec_out(start=324) "degC";
    input Real pumpRec_opening(start=0.35);

  // Calibrated parameters (input used for calibration in comment)

    // Gas Turbine
    output Units.FrictionCoefficient Filter_Kfr; // Filter outlet pressure
    output Real compression_rate; // Air compressor outlet pressure
    output Real compressor_eta_is; // Air compressor outlet temperature
    output Real turbine_eta_is; // Gas turbine power output
    // Economizer
    output Units.HeatExchangeCoefficient Eco_Kth; // Economizer water outlet temperature
    output Units.FrictionCoefficient Eco_Kfr_hot; // Gas turbine outlet pressure
    output Units.FrictionCoefficient Eco_Kfr_cold; // Economizer water outlet pressure
    // Evaporator
    output Units.Cv Evap_CV_Cvmax; // Evaporator control valve opening
    output Units.HeatExchangeCoefficient Evap_Kth; // Extraction pump mass flow rate
    // High Pressure Superheater 1
    output Units.HeatExchangeCoefficient HPSH1_Kth; // HP superheater outlet temperature
    output Units.FrictionCoefficient HPSH1_Kfr_cold; // HP superheater inlet pressure
    // High Pressure Superheater 2
    output Units.HeatExchangeCoefficient HPSH2_Kth; // De-superheater mass flow rate
    output Units.FrictionCoefficient HPSH2_Kfr_cold; // HP superheater inlet pressure
    // De-superheater
    output Units.Cv deSH_CV_Cvmax; // Desuperheater control valve opening
    // Reheater
    output Units.HeatExchangeCoefficient ReH_Kth; // LP superheater outlet temperature
    output Units.FrictionCoefficient ReH_Kfr_cold; // LP superheater inlet pressure
    // High Pressure Steam Turbine
    output Units.Cv HPST_CV_Cv; // HP superheater outlet pressure
    output Units.Cst HPST_Cst; // HP steam turbine inlet pressure
    output Units.Yield HPST_eta_is; // HP steam turbine outlet temperature
    output Units.Yield LPST_eta_is; // Power output
    // Low Pressure Steam Turbine
    output Units.Cv LPST_CV_Cv; // Low pressure superheater outlet pressure
    output Units.Cst LPST_Cst; // LP steam turbine inlet pressure
    // Condenser
    output Units.HeatExchangeCoefficient Cond_Kth; // Condensation pressure
    output Units.VolumeFlowRate Qv_cond_cold; // Circulating water outlet temperature
    // Exctraction Pump
    output Real pump_a3; // Exctraction pump outlet pressure
    output Real pump_b3; // Exctraction pump outlet temperature
    // Recirculation pump
    output Real pumpRec_a3; // Recirculation pump outlet pressure
    output Real pumpRec_b3; // Recirculation pump outlet temperature
    output Units.Cv pumpRec_CV_Cvmax;// Recirculation control valve opening

    // Observables of interest
    output Units.PositiveMassFlowRate Q_fuel_source; // Observable: controlled by the gas turbine outlet temperature
    output Real T_flue_gas_sink; // Observable
    output Real Q_pumpRec_out; // Observable: controlled by the economizer input temperature
    output Real turbine_compression_rate; // Observable of interest

  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Economiser economiser(
      QCp_max_side=Eco_QCp_max_side)
    annotation (Placement(transformation(extent={{74,-56},{132,3}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink flue_gas_sink
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={222,198})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_eco_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={8,8})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_eco_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={56,8})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Evaporator evaporator
    annotation (Placement(transformation(extent={{-46,-56},{12,4.5}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_evap_out_sensor
    annotation (Placement(transformation(extent={{-34,2},{-46,14}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater1(
      QCp_max_side=HPSH_QCp_max_side)
    annotation (Placement(transformation(extent={{-186,-56},{-126,4}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_HPSH1_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-184,8})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_HPSH1_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-208,8})));
  WaterSteam.Pipes.SlideValve                             HPST_control_valve
    annotation (Placement(transformation(extent={{-203.25,144.738},{-186.75,
            162.677}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_in_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-174,148})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPsteamTurbine annotation (Placement(transformation(extent={{-160,132},{-126,164}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_out_sensor
    annotation (Placement(transformation(extent={{-114,142},{-102,154}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_ST_out_sensor
    annotation (Placement(transformation(extent={{90,250},{102,262}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{32,144.778},{72,176.778}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_circulating_water_out_sensor
    annotation (Placement(transformation(extent={{86,171},{96,181}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source circulating_water_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-16,159})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink circulating_water_sink
    annotation (Placement(transformation(extent={{98,168},{114,184}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump annotation (Placement(
        transformation(
        extent={{-7,-7},{7,7}},
        origin={116,131},
        rotation=0)));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_pump_out_sensor
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={137,131})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_pump_out_sensor
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={155,
            131})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source powerSource
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={116,150})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={182,28})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_pump_out_sensor
    annotation (Placement(transformation(extent={{166,126},{176,136}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor(h_out(
        start=7e5))
    annotation (Placement(transformation(extent={{-524,-40},{-496,-12}})));
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine gasTurbine(
    eta_is(start=0.73),
    eta_mech(start=0.9),
    h_out(start=0.5e6))
    annotation (Placement(transformation(extent={{-414,-42},{-382,-10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_power
    annotation (Placement(transformation(extent={{-332,24},{-312,44}})));
  MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber(LHV=LHV_plant)
    annotation (Placement(transformation(extent={{-452,-36},{-432,-16}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source_fuel(h_out(
        start=0.9e6)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-442,-90})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor compressor_P_out_sensor
    annotation (Placement(transformation(extent={{-490,-32},{-478,-20}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor
    annotation (Placement(transformation(extent={{-472,-32},{-460,-20}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor turbine_P_out_sensor
    annotation (Placement(transformation(extent={{-350,-32},{-338,-20}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_GT_sensor
    annotation (Placement(transformation(extent={{-346,28},{-334,40}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor turbine_T_out_sensor
    annotation (Placement(transformation(extent={{-370,-32},{-358,-20}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater Reheater(
      QCp_max_side=ReH_QCp_max_side)
    annotation (Placement(transformation(extent={{-102,-56},{-42,4}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPsteamTurbine annotation (Placement(transformation(extent={{-14,198},{20,230}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_ReH_out_sensor
    annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=270,
        origin={-80,29})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_ReH_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-80,49})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_Cond_sensor
    annotation (Placement(transformation(extent={{28,208},{40,220}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_source_air_sensor
    annotation (Placement(transformation(extent={{-636,-32},{-624,-20}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_source_air_sensor
    annotation (Placement(transformation(extent={{-618,-32},{-606,-20}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlowSensor Q_source_air_sensor
    annotation (Placement(transformation(extent={{-600,-32},{-588,-20}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_circulating_water_in_sensor
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={3,159})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_circulating_water_in_sensor
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={19,159})));
  WaterSteam.Pipes.SlideValve                             LPST_control_valve
    annotation (Placement(transformation(extent={{-61.25,210.738},{-44.75,
            228.677}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_LPST_in_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-28,214})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump pumpRec(Q_0=1)
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        origin={94,48.5455},
        rotation=0)));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source pumpRec_powerSource
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={94,66})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_pumpRec_out_sensor
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={115,48.5455})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_pumpRec_out_sensor
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={131,
            48.5455})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_pumpRec_out_sensor
    annotation (Placement(transformation(extent={{140,43.5455},{150,53.5455}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_eco_in_sensor
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=180,
        origin={145,9})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve pumpRec_controlValve
    annotation (Placement(transformation(extent={{157,46},{170,60}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor pumpRec_opening_sensor
    annotation (Placement(transformation(extent={{158,68},{168,78}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_flue_gas_sink_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={222,172})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{110,246},{130,266}})));
  MetroscopeModelingLibrary.Sensors.Fuel.PressureSensor P_fuel_source_sensor
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-61})));
  MetroscopeModelingLibrary.Sensors.Fuel.TemperatureSensor T_fuel_source_sensor
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-75})));
  MetroscopeModelingLibrary.Sensors.Fuel.FlowSensor Q_fuel_source_sensor
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-47})));
  MetroscopeModelingLibrary.Power.Machines.Generator GT_generator
    annotation (Placement(transformation(extent={{-380,24},{-348,44}})));
  MetroscopeModelingLibrary.Power.Machines.Generator ST_generator
    annotation (Placement(transformation(extent={{50,246},{82,266}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_flue_gas_sink_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={170,-26})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Filter AirFilter
    annotation (Placement(transformation(extent={{-576,-36},{-556,-16}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_filter_out_sensor
    annotation (Placement(transformation(extent={{-548,-32},{-536,-20}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater2(
      QCp_max_side=HPSH_QCp_max_side)
    annotation (Placement(transformation(extent={{-302,-56},{-242,4}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_HPSH2_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-282,34})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_HPSH2_out_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-282,52})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve deSH_controlValve
    annotation (Placement(transformation(extent={{-158.75,89.4545},{-171.25,
            103.455}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor deSH_opening_sensor
    annotation (Placement(transformation(extent={{-170,114},{-160,124}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_deSH_sensor
    annotation (Placement(transformation(extent={{-132,86},{-144,98}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve Evap_controlValve
    annotation (Placement(transformation(extent={{41.25,5.4545},{28.75,19.455}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor Evap_opening_sensor
    annotation (Placement(transformation(extent={{30,34},{40,44}})));
  MultiFluid.Converters.MoistAir_to_FlueGases moistAir_to_FlueGases annotation (Placement(transformation(extent={{-672,-36},{-652,-16}})));
  MoistAir.BoundaryConditions.Source source_air(h_out(start=47645.766)) annotation (Placement(transformation(extent={{-708,-36},{-688,-16}})));
  Sensors.WaterSteam.TemperatureSensor T_HPST_out_sensor annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=180,
        origin={-90,148})));
equation

  //--- Air / Flue Gas System ---

    // Air source
      // Quantities definition
      P_source_air_sensor.P_barA = P_source_air;
      T_source_air_sensor.T_degC = T_source_air;
      Q_source_air_sensor.Q = Q_source_air;
      source_air.relative_humidity=Relative_Humidity;

    // Fuel Source
      //  Quantities definition
      Q_fuel_source_sensor.Q = Q_fuel_source;
      P_fuel_source_sensor.P_barA = P_fuel_source;
      T_fuel_source_sensor.T_degC = T_fuel_source;
      source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Air Filter
      // Quantities definition
      P_filter_out_sensor.P_barA = P_filter_out;
      // Calibrated parameters
      AirFilter.Kfr = Filter_Kfr;
      // Parameters
      AirFilter.delta_z = 1;

    // Air Compressor
      // Quantities definition
      compressor_T_out_sensor.T_degC = compressor_T_out;
      compressor_P_out_sensor.P_barA = compressor_P_out;
      // Calibrated parameters
      airCompressor.tau = compression_rate;
      airCompressor.eta_is = compressor_eta_is;

    // Combustion chamber
      // Parameters
      combustionChamber.Kfr = combustionChamber_Kfr;
      combustionChamber.eta = combustionChamber_eta;

    // Gas Turbine
      // Quantities definition
      gasTurbine.h_out = GT_h_out;
      turbine_P_out_sensor.P_barA = turbine_P_out;
      // Calibrated parameters
      gasTurbine.tau = turbine_compression_rate;
      gasTurbine.eta_is = turbine_eta_is;
      // Parameters
      gasTurbine.eta_mech = 0.99;

    // Generator
      // Quantities definition
      W_GT_sensor.W_MW = W_GT;
      // Parameters
      GT_generator.eta = 0.99;

    // Flue Gas sink
      //Quantities definition
      P_flue_gas_sink_sensor.P_barA = P_flue_gas_sink;
      T_flue_gas_sink_sensor.T_degC = T_flue_gas_sink;

  // --- Water / Steam Circuit

    // Economizer
      // Quantities definition
      T_w_eco_out_sensor.T_degC = T_w_eco_out;
      P_w_eco_out_sensor.P_barA = P_w_eco_out;
      // Parameters
      economiser.S = 100;
      economiser.nominal_cold_side_temperature_rise = 235;
      economiser.nominal_hot_side_temperature_drop = 150;
      T_w_eco_in_sensor.T_degC = T_w_eco_in;
      // Calibrated parameters
      economiser.Kth = Eco_Kth;
      economiser.Kfr_hot = Eco_Kfr_hot;
      economiser.Kfr_cold = Eco_Kfr_cold;

    // Evaporator
      // Quantities definition
      P_w_evap_out_sensor.P_barA = P_w_evap_out;
      evaporator.x_steam_out = Evap_x_steam_out;
      Evap_opening_sensor.Opening = Evap_opening;
      // Parameters
      evaporator.S = 100;
      evaporator.Kfr_hot = 0;
      // Calibrated parameters
      Evap_controlValve.Cvmax = Evap_CV_Cvmax;
      evaporator.Kth = Evap_Kth;
      evaporator.Kfr_cold = Evap_Kfr_cold;

    // HP Superheater 1
      // Quantities definition
      P_w_HPSH1_out_sensor.P_barA = P_w_HPSH1_out;
      T_w_HPSH1_out_sensor.T_degC = T_w_HPSH1_out;
      // Parameters
      HPsuperheater1.S = 100;
      HPsuperheater1.nominal_cold_side_temperature_rise = 250;
      HPsuperheater1.nominal_hot_side_temperature_drop = 180;
      HPsuperheater1.Kfr_hot = 0;
      // Calibrated parameters
      HPsuperheater1.Kth = HPSH1_Kth;
      HPsuperheater1.Kfr_cold = HPSH1_Kfr_cold;

    // HP Superheater 2
      // Quantities definition
      P_w_HPSH2_out_sensor.P_barA = P_w_HPSH2_out;
      // Parameters
      HPsuperheater2.S = 100;
      HPsuperheater2.nominal_cold_side_temperature_rise = 150;
      HPsuperheater2.nominal_hot_side_temperature_drop = 180;
      HPsuperheater2.Kfr_hot = 0;
      T_w_HPSH2_out_sensor.T_degC = T_w_HPSH2_out;
      // Calibrated parameters
      HPsuperheater2.Kth = HPSH2_Kth;
      HPsuperheater2.Kfr_cold = HPSH2_Kfr_cold;

    // De-superheater
      // Quantities definition
      deSH_opening_sensor.Opening = deSH_opening;
      Q_deSH_sensor.Q = Q_deSH;
      // Calibrated parameters
      deSH_controlValve.Cvmax = deSH_CV_Cvmax;

    // Reheater
      // Quantities definition
      T_w_ReH_out_sensor.T_degC = T_w_ReH_out;
      P_w_ReH_out_sensor.P_barA = P_w_ReH_out;
      // Parameters
      Reheater.S = 100;
      Reheater.nominal_cold_side_temperature_rise = 100;
      Reheater.nominal_hot_side_temperature_drop = 180;
      Reheater.Kfr_hot = 0;
      // Calibrated parameters
      Reheater.Kth = ReH_Kth;
      Reheater.Kfr_cold = ReH_Kfr_cold;

    // Steam Turbines

      // Steam Turbine Generator
        // Quantities definition
        W_ST_out_sensor.W_MW = W_ST_out;
        // Parameters
        ST_generator.eta = 0.99;

      // High Pressure Level
        // Quantities definition
        P_HPST_in_sensor.P_barA = P_ST_in;
        P_HPST_out_sensor.P_barA = P_ST_out;
        T_HPST_out_sensor.T_degC = T_HPST_out;
        // Calibrated Parameters
        HPST_control_valve.Cv = HPST_CV_Cv;
        HPsteamTurbine.Cst = HPST_Cst;
        HPsteamTurbine.eta_is = HPST_eta_is;

      // Low Pressure Level
        // Quantities definition
        P_LPST_in_sensor.P_barA = P_LPST_in;
        // Calibrated Parameters
        LPsteamTurbine.eta_is = LPST_eta_is;
        LPST_control_valve.Cv = LPST_CV_Cv;
        LPsteamTurbine.Cst = LPST_Cst;

    // Condenser
      // Quantities definition
      P_circulating_water_in_sensor.P_barA = P_circulating_water_in;
      T_circulating_water_in_sensor.T_degC = T_circulating_water_in;
      T_circulating_water_out_sensor.T_degC = T_circulating_water_out;
      P_Cond_sensor.P_barA   = P_Cond;
      // Parameters
      condenser.S = 100;
      condenser.water_height = 1;
      condenser.C_incond = 0;
      condenser.P_offset = 0;
      condenser.Kfr_cold = 1;
      // Calibrated parameters
      condenser.Kth = Cond_Kth;
      condenser.Qv_cold_in = Qv_cond_cold;

    // Exctraction Pump
      // Quantities definition
      T_pump_out_sensor.T_degC = T_pump_out;
      P_pump_out_sensor.P_barA = P_pump_out;
      Q_pump_out_sensor.Q = Q_pump_out;
      // Parameters
      pump.VRot = 1400;
      pump.VRotn = 1400;
      pump.rm = 0.85;
      pump.a1 = 0;
      pump.a2 = 0;
      pump.b1 = 0;
      pump.b2 = 0;
      pump.rhmin = 0.0001;
      // Calibrated parameters
      pump.a3 = pump_a3;
      pump.b3 = pump_b3;

    //--- Recirculation pump ---
      // Quantities definition
      T_pumpRec_out_sensor.T_degC = T_pumpRec_out;
      P_pumpRec_out_sensor.P_barA = P_pumpRec_out;
      pumpRec_opening_sensor.Opening = pumpRec_opening;
      Q_pumpRec_out_sensor.Q = Q_pumpRec_out;
      // Parameters
      pumpRec.VRot = 1400;
      pumpRec.VRotn = 1400;
      pumpRec.rm = 0.85;
      pumpRec.a1 = 0;
      pumpRec.a2 = 0;
      pumpRec.b1 = 0;
      pumpRec.b2 = 0;
      pumpRec.rhmin = 0.0001;
      // Calibrated parameters
      pumpRec.a3 = pumpRec_a3;
      pumpRec.b3 = pumpRec_b3;
      pumpRec_controlValve.Cvmax = pumpRec_CV_Cvmax;

  connect(HPsuperheater1.C_cold_out, T_w_HPSH1_out_sensor.C_in) annotation (
     Line(points={{-165,-5},{-166,-5},{-166,8},{-178,8}}, color={28,108,200}));
  connect(P_HPST_out_sensor.C_in, HPsteamTurbine.C_out)
    annotation (Line(points={{-114,148},{-126,148}}, color={28,108,200}));
  connect(P_HPST_in_sensor.C_out, HPsteamTurbine.C_in)
    annotation (Line(points={{-168,148},{-160,148}}, color={28,108,200}));
  connect(T_circulating_water_out_sensor.C_out, circulating_water_sink.C_in)
    annotation (Line(points={{96,176},{102,176}},color={28,108,200}));
  connect(combustionChamber.outlet,gasTurbine. C_in) annotation (Line(points={{-432,
          -26},{-414,-26}},                                                                   color={95,95,95}));
  connect(airCompressor.C_out,compressor_P_out_sensor. C_in) annotation (Line(points={{-496,
          -26},{-490,-26}},                                                                             color={95,95,95}));
  connect(compressor_P_out_sensor.C_out,compressor_T_out_sensor. C_in) annotation (Line(points={{-478,
          -26},{-472,-26}},                                                                                       color={95,95,95}));
  connect(compressor_T_out_sensor.C_out,combustionChamber. inlet) annotation (Line(points={{-460,
          -26},{-452,-26}},                                                                                  color={95,95,95}));
  connect(evaporator.C_cold_in, T_w_eco_out_sensor.C_out) annotation (Line(
        points={{-8.3,-4.575},{-8.3,8},{2,8}},                 color={28,108,200}));
  connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in)
    annotation (Line(points={{71.6,159},{74,159},{74,158},{78,158},{78,176},{86,176}},
                    color={28,108,200}));
  connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{52,144.778},{52,131},{109,131}},
                              color={28,108,200}));
  connect(powerSource.C_out, pump.C_power)
    annotation (Line(points={{116,145.2},{116,138.56}},
                                                   color={244,125,35}));
  connect(HPsuperheater1.C_cold_in, P_w_evap_out_sensor.C_out) annotation (Line(
        points={{-147,-5},{-147,8},{-46,8}},  color={28,108,200}));
  connect(evaporator.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
        points={{-25.7,-4.575},{-24,-4.575},{-24,8},{-34,8}},
                                                        color={28,108,200}));
  connect(economiser.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
        points={{94.3,-5.85},{94,-5.85},{94,8},{62,8}},
                                                   color={28,108,200}));
  connect(evaporator.C_hot_out, economiser.C_hot_in) annotation (Line(points={{3.3,
          -26.355},{51,-26.355},{51,-26.5},{82.7,-26.5}},color={95,95,95}));
  connect(gasTurbine.C_out, turbine_T_out_sensor.C_in)
    annotation (Line(points={{-382,-26},{-370,-26}}, color={95,95,95}));
  connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out)
    annotation (Line(points={{-350,-26},{-358,-26}}, color={95,95,95}));
  connect(evaporator.C_hot_in, Reheater.C_hot_out) annotation (Line(points={{-37.3,
          -26.355},{-37.3,-26},{-51,-26}}, color={95,95,95}));
  connect(HPsuperheater1.C_hot_out, Reheater.C_hot_in)
    annotation (Line(points={{-135,-26},{-93,-26}}, color={95,95,95}));
  connect(Reheater.C_cold_out, T_w_ReH_out_sensor.C_in)
    annotation (Line(points={{-81,-5},{-80,-5},{-80,23}}, color={28,108,200}));
  connect(P_Cond_sensor.C_in, LPsteamTurbine.C_out)
    annotation (Line(points={{28,214},{20,214}},   color={28,108,200}));
  connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,214},{52,214},{52,177.134}},
                                     color={28,108,200}));

  connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
    annotation (Line(points={{-624,-26},{-618,-26}}, color={95,95,95}));
  connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
    annotation (Line(points={{-606,-26},{-600,-26}}, color={95,95,95}));
  connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
     Line(points={{32,159},{30,159},{30,159},{24,159}},               color={28,
          108,200}));
  connect(P_LPST_in_sensor.C_out, LPsteamTurbine.C_in)
    annotation (Line(points={{-22,214},{-14,214}},  color={28,108,200}));
  connect(P_LPST_in_sensor.C_in, LPST_control_valve.C_out) annotation (Line(
        points={{-34,214},{-40,214},{-40,214},{-44.75,214}},             color={
          28,108,200}));
  connect(P_w_ReH_out_sensor.C_out, LPST_control_valve.C_in) annotation (Line(
        points={{-80,55},{-80,214},{-61.25,214}}, color={28,108,200}));
  connect(P_w_ReH_out_sensor.C_in, T_w_ReH_out_sensor.C_out)
    annotation (Line(points={{-80,43},{-80,35}}, color={28,108,200}));
  connect(T_w_HPSH1_out_sensor.C_out, P_w_HPSH1_out_sensor.C_in)
    annotation (Line(points={{-190,8},{-202,8}}, color={28,108,200}));
  connect(HPST_control_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
        points={{-186.75,148},{-184,148},{-184,148},{-180,148}}, color={28,108,200}));
  connect(T_circulating_water_in_sensor.C_out, P_circulating_water_in_sensor.C_in)
    annotation (Line(points={{8,159},{14,159}},    color={28,108,200}));
  connect(circulating_water_source.C_out, T_circulating_water_in_sensor.C_in)
    annotation (Line(points={{-11,159},{-2,159}},            color={28,108,200}));
  connect(pump.C_out, T_pump_out_sensor.C_in)
    annotation (Line(points={{123,131},{132,131}},     color={28,108,200}));
  connect(T_pump_out_sensor.C_out, P_pump_out_sensor.C_in) annotation (Line(
        points={{142,131},{150,131}},             color={28,108,200}));
  connect(P_pump_out_sensor.C_out, Q_pump_out_sensor.C_in) annotation (Line(
        points={{160,131},{166,131}},             color={28,108,200}));
  connect(pumpRec.C_power, pumpRec_powerSource.C_out)
    annotation (Line(points={{94,56.1055},{94,61.2}},
                                                    color={244,125,35}));
  connect(P_pumpRec_out_sensor.C_out, Q_pumpRec_out_sensor.C_in)
    annotation (Line(points={{136,48.5455},{140,48.5455}},
                                               color={28,108,200}));
  connect(pumpRec_controlValve.Opening, pumpRec_opening_sensor.Opening)
    annotation (Line(points={{163.5,58.7273},{163.5,62},{163,62},{163,67.9}},
                                                                color={0,0,127}));
  connect(pumpRec.C_in, P_w_eco_out_sensor.C_in) annotation (Line(points={{87,48.5455},
          {74,48.5455},{74,8},{62,8}},
                                   color={28,108,200}));
  connect(economiser.C_cold_in, T_w_eco_in_sensor.C_out) annotation (Line(
        points={{111.7,-5.85},{111.7,9},{140,9}},
                                              color={28,108,200}));
  connect(T_w_eco_in_sensor.C_in, loopBreaker.C_out) annotation (Line(points={{150,9},
          {150,8},{182,8},{182,18}},   color={28,108,200}));
  connect(Q_pump_out_sensor.C_out, loopBreaker.C_in)
    annotation (Line(points={{176,131},{182,131},{182,38}},
                                                         color={28,108,200}));
  connect(T_pumpRec_out_sensor.C_out, P_pumpRec_out_sensor.C_in) annotation (
      Line(points={{120,48.5455},{126,48.5455}},                         color={
          28,108,200}));
  connect(Q_pumpRec_out_sensor.C_out, pumpRec_controlValve.C_in)
    annotation (Line(points={{150,48.5455},{154,48.5455},{154,48.5455},{157,48.5455}},
                                                         color={28,108,200}));
  connect(flue_gas_sink.C_in, P_flue_gas_sink_sensor.C_out)
    annotation (Line(points={{222,193},{222,178}}, color={95,95,95}));
  connect(sink.C_in, W_ST_out_sensor.C_out)
    annotation (Line(points={{115,256},{101.88,256}},color={244,125,35}));
  connect(combustionChamber.inlet1, Q_fuel_source_sensor.C_out)
    annotation (Line(points={{-442,-36},{-442,-42}}, color={213,213,0}));
  connect(Q_fuel_source_sensor.C_in, P_fuel_source_sensor.C_out) annotation (
      Line(points={{-442,-52},{-442,-54},{-442,-54},{-442,-56}}, color={213,213,
          0}));
  connect(P_fuel_source_sensor.C_in, T_fuel_source_sensor.C_out) annotation (
      Line(points={{-442,-66},{-442,-68},{-442,-68},{-442,-70}}, color={213,213,
          0}));
  connect(source_fuel.C_out, T_fuel_source_sensor.C_in)
    annotation (Line(points={{-442,-85},{-442,-80}}, color={213,213,0}));
  connect(GT_generator.C_out, W_GT_sensor.C_in)
    annotation (Line(points={{-352.8,34},{-346,34}}, color={244,125,35}));
  connect(W_GT_sensor.C_out, sink_power.C_in)
    annotation (Line(points={{-334.12,34},{-327,34}}, color={244,125,35}));
  connect(GT_generator.C_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-373.92,34},{-382,34},{-382,-10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(W_ST_out_sensor.C_in, ST_generator.C_out)
    annotation (Line(points={{90,256},{77.2,256}},color={244,125,35}));
  connect(HPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{-126,161.44},{-126,161.44},{-126,194},{-126,256},{56.08,256}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{20,227.44},{20,256},{56.08,256}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(pumpRec_controlValve.C_out, loopBreaker.C_in) annotation (Line(points={{170,48.5455},{174,48.5455},{174,50},{182,50},{182,38}},
                                                                color={28,108,200}));
  connect(pumpRec.C_out, T_pumpRec_out_sensor.C_in) annotation (Line(points={{101,
          48.5455},{110,48.5455}},             color={28,108,200}));
  connect(AirFilter.C_out, P_filter_out_sensor.C_in)
    annotation (Line(points={{-556,-26},{-548,-26}}, color={95,95,95}));
  connect(Q_source_air_sensor.C_out, AirFilter.C_in)
    annotation (Line(points={{-588,-26},{-576,-26}}, color={95,95,95}));
  connect(P_filter_out_sensor.C_out, airCompressor.C_in)
    annotation (Line(points={{-536,-26},{-524,-26}}, color={95,95,95}));
  connect(HPsuperheater1.C_hot_in, HPsuperheater2.C_hot_out)
    annotation (Line(points={{-177,-26},{-251,-26}}, color={95,95,95}));
  connect(P_w_HPSH1_out_sensor.C_out, HPsuperheater2.C_cold_in) annotation (
     Line(points={{-214,8},{-263,8},{-263,-5}}, color={28,108,200}));
  connect(P_w_HPSH2_out_sensor.C_out, HPST_control_valve.C_in) annotation (Line(
        points={{-282,58},{-282,148},{-203.25,148}}, color={28,108,200}));
  connect(turbine_P_out_sensor.C_out, HPsuperheater2.C_hot_in)
    annotation (Line(points={{-338,-26},{-293,-26}}, color={95,95,95}));
  connect(deSH_opening_sensor.Opening, deSH_controlValve.Opening)
    annotation (Line(points={{-165,113.9},{-165,102.182}}, color={0,0,127}));
  connect(Q_deSH_sensor.C_in, loopBreaker.C_in) annotation (Line(points={{-132,92},
          {182,92},{182,38}}, color={28,108,200}));
  connect(Q_deSH_sensor.C_out, deSH_controlValve.C_in) annotation (Line(points={{-144,92},{-152,92},{-152,92},{-158.75,92}},
                                                       color={28,108,200}));
  connect(deSH_controlValve.C_out, HPsuperheater2.C_cold_in) annotation (Line(
        points={{-171.25,92},{-230,92},{-230,8},{-263,8},{-263,-5}}, color={28,108,
          200}));
  connect(T_w_HPSH2_out_sensor.C_in, HPsuperheater2.C_cold_out) annotation (
      Line(points={{-282,28},{-282,12},{-282,-5},{-281,-5}}, color={28,108,200}));
  connect(P_w_HPSH2_out_sensor.C_in, T_w_HPSH2_out_sensor.C_out)
    annotation (Line(points={{-282,46},{-282,40}}, color={28,108,200}));
  connect(P_w_eco_out_sensor.C_out, Evap_controlValve.C_in) annotation (Line(
        points={{50,8},{45.625,8},{45.625,8.00005},{41.25,8.00005}}, color={28,108,
          200}));
  connect(T_w_eco_out_sensor.C_in, Evap_controlValve.C_out) annotation (Line(
        points={{14,8},{21.375,8},{21.375,8.00005},{28.75,8.00005}}, color={28,108,
          200}));
  connect(Evap_controlValve.Opening, Evap_opening_sensor.Opening)
    annotation (Line(points={{35,18.1822},{35,33.9}}, color={0,0,127}));
  connect(economiser.C_hot_out, T_flue_gas_sink_sensor.C_in) annotation (Line(
        points={{123.3,-26.5},{123.3,-26},{164,-26}}, color={95,95,95}));
  connect(T_flue_gas_sink_sensor.C_out, P_flue_gas_sink_sensor.C_in)
    annotation (Line(points={{176,-26},{222,-26},{222,166}}, color={95,95,95}));
  connect(P_source_air_sensor.C_in, moistAir_to_FlueGases.outlet) annotation (Line(points={{-636,-26},{-652,-26}}, color={95,95,95}));
  connect(moistAir_to_FlueGases.inlet, source_air.C_out) annotation (Line(points={{-672,-26},{-693,-26}}, color={85,170,255}));
  connect(P_HPST_out_sensor.C_out, T_HPST_out_sensor.C_in) annotation (Line(points={{-102,148},{-96,148}}, color={28,108,200}));
  connect(T_HPST_out_sensor.C_out, Reheater.C_cold_in) annotation (Line(points={{-84,148},{-63,148},{-63,-5}}, color={28,108,200}));
  connect(airCompressor.C_W_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-496,-12},{-496,8},{-382,8},{-382,-10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-720,-120},{260,280}})),
                                                              Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-720,-120},{260,280}}),
        graphics={Rectangle(
          extent={{-324,18},{246,-72}},
          pattern=LinePattern.None,
          lineColor={0,0,0},
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid), Text(
          extent={{-92,-56},{54,-68}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          textString="Heat Recovery Steam Generator"),
        Polygon(
          points={{-380,-10},{-380,-10},{-380,-42},{-324,-72},{-324,18},{-380,-10}},
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{246,16},{200,190}},
          pattern=LinePattern.None,
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-638,-18},{-586,-34}},
          fillColor={255,82,82},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-14,7},{14,-7}},
          fillColor={255,82,82},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={-443,-68},
          rotation=-90),
        Rectangle(
          extent={{-15,7},{15,-7}},
          fillColor={255,82,82},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={11,159},
          rotation=360),
        Rectangle(
          extent={{-8,8},{8,-8}},
          fillColor={255,82,82},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={222,172},
          rotation=360),
        Rectangle(
          extent={{-372,-18},{-356,-34}},
          pattern=LinePattern.None,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Rectangle(
          extent={{138,16},{152,2}},
          pattern=LinePattern.None,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-450,-40},{-436,-54}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={244,237,30},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{138,56},{152,42}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={244,237,30},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{162,-18},{178,-34}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={244,237,30},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-290,42},{-274,26}},
          pattern=LinePattern.None,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-230,96},{-174,96}},
          textColor={28,108,200},
          textString="Desuperheater"),
        Rectangle(
          extent={{-10,10},{10,-10}},
          fillColor={255,82,82},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={-670,248},
          rotation=360),
        Text(
          extent={{-650,253},{-564,243}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          textString="Boundary Conditions",
          fontSize=8),
        Rectangle(
          extent={{-680,232},{-660,212}},
          pattern=LinePattern.None,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-650,227},{-564,217}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          textString="Control Parameters",
          fontSize=8),
        Rectangle(
          extent={{-680,206},{-660,186}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={244,237,30},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-650,204},{-476,188}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="Observables not used for calibration"),
        Line(
          points={{-687,164},{-653,164}},
          color={95,95,95},
          thickness=0.5),
        Text(
          extent={{-634,169},{-548,159}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="Flue Gas flow"),
        Line(
          points={{-687,132},{-653,132}},
          color={238,46,47},
          thickness=0.5),
        Text(
          extent={{-634,137},{-548,127}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="HP flow"),
        Line(
          points={{-687,118},{-653,118}},
          color={244,125,35},
          thickness=0.5),
        Text(
          extent={{-634,123},{-548,113}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="IP flow"),
        Line(
          points={{-687,104},{-653,104}},
          color={244,237,30},
          thickness=0.5),
        Text(
          extent={{-634,109},{-548,99}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="LP flow"),
        Text(
          extent={{-590,125},{-504,115}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=7,
          textString="Solid Line: Liquid Phase
Dashed Line: Vapor Phase"),
        Text(
          extent={{-614,149},{-528,139}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          fontSize=8,
          textString="Water/Steam"),
        Rectangle(extent={{-696,152},{-490,94}}, lineColor={0,0,0}),
                                          Text(
          extent={{-306,-48},{-244,-54}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Superheater 2"),    Text(
          extent={{-186,-48},{-124,-54}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Superheater 1"),    Text(
          extent={{-104,-48},{-42,-54}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Reheater"),         Text(
          extent={{-48,-48},{14,-54}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Evaporator"),       Text(
          extent={{74,-52},{136,-58}},
          textColor={0,0,0},
          textStyle={TextStyle.Bold},
          fontSize=6,
          textString="Economizer
")}));
end MetroscopiaCCGT_causality_reverse;
