within MetroscopeModelingLibrary.Examples;
package CCGT
  package MetroscopiaCCGT

    model MetroscopiaCCGT_causality_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

      // Boundary conditions

        // Air source
        input MetroscopeModelingLibrary.Units.Pressure P_source_air(start=1) "bar";
        input MetroscopeModelingLibrary.Units.NegativeMassFlowRate Q_source_air(start=500) "kg/s";
        input Real T_source_air(start=24) "degC";
        // Fuel source
        input MetroscopeModelingLibrary.Units.Pressure P_fuel_source(start=30) "bar";
        input Real T_fuel_source(start=156);
        // Circulating water circuit
        input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
        input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
        // Flue gas sink
        input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";

      // Parameters

        // Gas Turbine
        parameter MetroscopeModelingLibrary.Units.SpecificEnthalpy LHV=48130e3;
        parameter Real GT_h_out = 1e6; // This enthalpy corresponds to T = 640°C at 1.1 bar 1e6
        // Economizer
        parameter String Eco_QCp_max_side = "hot";
        parameter Real T_w_eco_in = 85; // Controlled by the economizer recirculation pump flow rate
        // Evaporator
        parameter Real Evap_x_steam_out=1;
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient Evap_Kfr_cold=0;
        // High Pressure Superheater
        parameter String HPSH_QCp_max_side = "hot";
        // High Pressure Superheater 2
        parameter Real T_w_HPSH2_out = 566.5 "degC"; // Controlled by the desuperheater mass flow rate
        // Low Pressure Superheater (resuperheater)
        parameter String ReH_QCp_max_side = "hot";
        // Steam turbines control valves
        parameter Real LPST_opening = 1;
        parameter Real HPST_opening = 1;

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
        input Real Q_deSH(start=2);
        // Reheater
        input Real T_w_ReH_out(start = 350, min = 0, nominal = 350) "degC";
        input Real P_w_ReH_out(start=9, min=0, nominal=9) "barA";
        // High Pressure Steam Turbine
        input Real P_ST_in(start=113);
        input Real P_ST_out(start=10);
        //input Real P_ST_out(start=10, unit="bar", nominal=10, min=0) "bar";
        input Real W_ST_out(start=65, unit="MW", nominal=65, min=0) "MW";
        input Real P_LPST_in(start=8, min=0, nominal=4.9) "bar";
        // Condenser
        input Real P_Cond(start=0.05, min=0, nominal=0.05) "bar";
        input Real T_circulating_water_out(start=25, min=10, nominal=25) "degC";
        // Extraction Pump
        input Real P_pump_out(start=170) "barA";
        input Real T_pump_out(start=35) "degC";
        input Real Q_pump_out(start=50);
        // Recirculation Pump
        input Real P_pumpRec_out(start=180) "barA";
        input Real T_pumpRec_out(start=324) "degC";
        input Real pumpRec_opening(start=0.35);

      // Calibrated parameters (input used for calibration in comment)

        // Gas Turbine
        output MetroscopeModelingLibrary.Units.FrictionCoefficient Filter_Kfr; // Filter outlet pressure
        output Real compression_rate; // Air compressor outlet pressure
        output Real compressor_eta_is; // Air compressor outlet temperature
        output Real turbine_eta_is; // Gas turbine power output
        // Economizer
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Eco_Kth; // Economizer water outlet temperature
        output MetroscopeModelingLibrary.Units.FrictionCoefficient Eco_Kfr_hot; // Gas turbine outlet pressure
        output MetroscopeModelingLibrary.Units.FrictionCoefficient Eco_Kfr_cold; // Economizer water outlet pressure
        // Evaporator
        output MetroscopeModelingLibrary.Units.Cv Evap_CV_Cvmax; // Evaporator control valve opening
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Evap_Kth; // Extraction pump mass flow rate
        // High Pressure Superheater 1
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient HPSH1_Kth; // HP superheater outlet temperature
        output MetroscopeModelingLibrary.Units.FrictionCoefficient HPSH1_Kfr_cold; // HP superheater inlet pressure
        // High Pressure Superheater 2
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient HPSH2_Kth; // De-superheater mass flow rate
        output MetroscopeModelingLibrary.Units.FrictionCoefficient HPSH2_Kfr_cold; // HP superheater inlet pressure
        // De-superheater
        output MetroscopeModelingLibrary.Units.Cv deSH_CV_Cvmax; // Desuperheater control valve opening
        // Reheater
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient ReH_Kth; // LP superheater outlet temperature
        output MetroscopeModelingLibrary.Units.FrictionCoefficient ReH_Kfr_cold; // LP superheater inlet pressure
        // High Pressure Steam Turbine
        output MetroscopeModelingLibrary.Units.Cv HPST_CV_Cvmax; // HP superheater outlet pressure
        output MetroscopeModelingLibrary.Units.Cst HPST_Cst; // HP steam turbine inlet pressure
        output MetroscopeModelingLibrary.Units.Yield ST_eta_is; // Power output
        // Low Pressure Steam Turbine
        output MetroscopeModelingLibrary.Units.Cv LPST_CV_Cvmax; // Low pressure superheater outlet pressure
        output MetroscopeModelingLibrary.Units.Cst LPST_Cst; // LP steam turbine inlet pressure
        // Condenser
        output MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Cond_Kth; // Condensation pressure
        output MetroscopeModelingLibrary.Units.VolumeFlowRate Qv_cond_cold; // Circulating water outlet temperature
        // Exctraction Pump
        output Real pump_a3; // Exctraction pump outlet pressure
        output Real pump_b3; // Exctraction pump outlet temperature
        // Recirculation pump
        output Real pumpRec_a3; // Recirculation pump outlet pressure
        output Real pumpRec_b3; // Recirculation pump outlet temperature
        output Real pumpRec_CV_Cvmax; // Recirculation control valve opening

        // Observables of interest

        output MetroscopeModelingLibrary.Units.NegativeMassFlowRate Q_fuel_source; // Observable: controlled by the gas turbine outlet temperature 10.5
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
      MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HPST_control_valve
        annotation (Placement(transformation(extent={{-203.25,144.738},{-186.75,
                162.677}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-174,148})));
      MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine HPsteamTurbine
        annotation (Placement(transformation(extent={{-160,132},{-126,164}})));
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
      MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source_air(h_out(
            start=0.3e6))
        annotation (Placement(transformation(extent={{-658,-36},{-638,-16}})));
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
      MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber
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
      MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LPsteamTurbine
        annotation (Placement(transformation(extent={{-14,198},{20,230}})));
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
      MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LPST_control_valve
        annotation (Placement(transformation(extent={{-61.25,210.738},{-44.75,
                228.677}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_LPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-28,214})));
      MetroscopeModelingLibrary.WaterSteam.Machines.Pump pumpRec(Q_in_0=1)
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
    equation

      //--- Air / Flue Gas System ---

        // Air source
          // Quantities definition
          P_source_air_sensor.P_barA = P_source_air;
          T_source_air_sensor.T_degC = T_source_air;
          Q_source_air_sensor.Q = Q_source_air;
          source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

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
          combustionChamber.LHV = LHV;
          combustionChamber.DP = 0.1e5;

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
          economiser.nominal_hot_side_temperature_rise = 150;
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
          evaporator.S_vaporising = 100;
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
          HPsuperheater1.nominal_hot_side_temperature_rise = 180;
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
          HPsuperheater2.nominal_hot_side_temperature_rise = 180;
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
          Reheater.nominal_hot_side_temperature_rise = 180;
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
            // Parameters
            HPST_control_valve.Opening = HPST_opening;
            HPsteamTurbine.area_nz = 1;
            HPsteamTurbine.eta_nz = 1;
            HPsteamTurbine.eta_is = LPsteamTurbine.eta_is;
            // Calibrated Parameters
            HPST_control_valve.Cvmax = HPST_CV_Cvmax;
            HPsteamTurbine.Cst = HPST_Cst;
            HPsteamTurbine.eta_is = ST_eta_is;

          // Low Pressure Level
            // Quantities definition
            P_LPST_in_sensor.P_barA = P_LPST_in;
            // Parameters
            LPST_control_valve.Opening = LPST_opening;
            LPsteamTurbine.area_nz = 1;
            LPsteamTurbine.eta_nz = 1;
            // Calibrated Parameters
            LPST_control_valve.Cvmax = LPST_CV_Cvmax;
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
        annotation (Line(points={{72,157.934},{74,157.934},{74,158},{78,158},{78,176},{86,176}},
                        color={28,108,200}));
      connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{52,144.067},{52,131},{109,131}},
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
      connect(gasTurbine.C_W_compressor, airCompressor.C_W_in) annotation (Line(
          points={{-414,-10},{-414,16},{-496,16},{-496,-12}},
          color={244,125,35},
          smooth=Smooth.Bezier));
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
      connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,214},
              {52,214},{52,176.778}},    color={28,108,200}));

      connect(P_HPST_out_sensor.C_out, Reheater.C_cold_in) annotation (Line(points={
              {-102,148},{-63,148},{-63,-5}}, color={28,108,200}));
      connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
        annotation (Line(points={{-624,-26},{-618,-26}}, color={95,95,95}));
      connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
        annotation (Line(points={{-606,-26},{-600,-26}}, color={95,95,95}));
      connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
         Line(points={{31.2,167.889},{30,167.889},{30,159},{24,159}},     color={28,
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
      connect(GT_generator.C_in, gasTurbine.C_W_out) annotation (Line(
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
      connect(P_source_air_sensor.C_in, source_air.C_out)
        annotation (Line(points={{-636,-26},{-643,-26}}, color={95,95,95}));
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-680,-120},
                {260,280}})),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-680,-120},{260,280}}),
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
              extent={{-10,10},{10,-10}},
              fillColor={255,82,82},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              origin={-570,210},
              rotation=360),
            Text(
              extent={{-552,214},{-466,204}},
              textColor={0,0,0},
              textString="Boundary Conditions",
              horizontalAlignment=TextAlignment.Left),
            Rectangle(
              extent={{-580,180},{-560,160}},
              pattern=LinePattern.None,
              fillColor={0,140,72},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,0}),
            Text(
              extent={{-552,176},{-466,166}},
              textColor={0,0,0},
              textString="Parameters",
              horizontalAlignment=TextAlignment.Left),
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
              extent={{-580,140},{-560,120}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={244,237,30},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-552,138},{-420,122}},
              textColor={0,0,0},
              horizontalAlignment=TextAlignment.Left,
              textString="Observables not used for calibration"),
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
              textString="Desuperheater")}));
    end MetroscopiaCCGT_causality_reverse;

    model MetroscopiaCCGT_causality_direct
      extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

      // Boundary conditions

        // Air source
        input MetroscopeModelingLibrary.Units.Pressure P_source_air(start=1) "bar";
        input MetroscopeModelingLibrary.Units.NegativeMassFlowRate Q_source_air(start=500) "kg/s";
        input Real T_source_air(start=24) "degC";
        // Fuel source
        input MetroscopeModelingLibrary.Units.Pressure P_fuel_source(start=30) "bar";
        input Real T_fuel_source(start=156);
        // Circulating water circuit
        input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
        input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
        // Flue gas sink
        input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";

      // Parameters

        // Gas Turbine
        parameter MetroscopeModelingLibrary.Units.SpecificEnthalpy LHV=48130e3;
        parameter Real turbine_T_out = 640;
        // Economizer
        parameter String Eco_QCp_max_side = "hot";
        parameter Real T_w_eco_in = 85; // Controlled by the economizer recirculation pump flow rate
        // Evaporator
        parameter Real Evap_x_steam_out=1;
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient Evap_Kfr_cold=0;
        // High Pressure Superheater
        parameter String HPSH_QCp_max_side = "hot";
        // High Pressure Superheater 2
        parameter Real T_w_HPSH2_out = 566.5 "degC"; // Controlled by the desuperheater mass flow rate
        // Low Pressure Superheater (resuperheater)
        parameter String ReH_QCp_max_side = "hot";
        // Steam turbines control valves
        parameter Real LPST_opening = 1;
        parameter Real HPST_opening = 1;

      // Observables used for calibration

        // Gas Turbine
        output Real P_filter_out;
        output Real compressor_P_out;
        output Real compressor_T_out;
        output Real W_GT;
        output Real turbine_P_out;
        // Economizer
        output Real P_w_eco_out;
        output Real T_w_eco_out;
        // Evaporator
        output Real Evap_opening;
        output Real P_w_evap_out;
        // High Pressure Superheater 1
        output Real P_w_HPSH1_out;
        output Real T_w_HPSH1_out;
        // High Pressure Superheater 2
        output Real P_w_HPSH2_out;
        // De-superheater
        output Real deSH_opening;
        output Real Q_deSH;
        // Reheater
        output Real T_w_ReH_out;
        output Real P_w_ReH_out;
        // High Pressure Steam Turbine
        output Real P_ST_in;
        output Real P_ST_out;
        output Real W_ST_out;
        output Real P_LPST_in;
        // Condenser
        output Real P_Cond;
        output Real T_circulating_water_out;
        // Extraction Pump
        output Real P_pump_out;
        output Real T_pump_out;
        output Real Q_pump_out;
        // Recirculation Pump
        output Real P_pumpRec_out;
        output Real T_pumpRec_out;
        output Real pumpRec_opening;

      // Calibrated parameters (input used for calibration in comment)

        // Gas Turbine
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient Filter_Kfr = 0.04432005;// Filter outlet pressure
        parameter Real compression_rate = 18.88889; // Air compressor outlet pressure
        parameter Real compressor_eta_is = 0.88200104; // Air compressor outlet temperature
        parameter Real turbine_eta_is = 0.8269205; // Gas turbine power output
        // Economizer
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Eco_Kth = 3403.8103; // Economizer water outlet temperature
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient Eco_Kfr_hot = 0.018985651; // Gas turbine outlet pressure
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient Eco_Kfr_cold = 973146.4; // Economizer water outlet pressure
        // Evaporator
        parameter MetroscopeModelingLibrary.Units.Cv Evap_CV_Cvmax = 539.1173; // Evaporator control valve opening
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Evap_Kth = 3279.242; // Extraction pump mass flow rate
        // High Pressure Superheater 1
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient HPSH1_Kth = 1181.761; // HP superheater outlet temperature
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient HPSH1_Kfr_cold = 7030.31; // HP superheater inlet pressure
        // High Pressure Superheater 2
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient HPSH2_Kth = 1661.5535; // De-superheater mass flow rate
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient HPSH2_Kfr_cold = 2538.3271; // HP superheater inlet pressure
        // De-superheater
        parameter MetroscopeModelingLibrary.Units.Cv deSH_CV_Cvmax = 7.7502966; // Desuperheater control valve opening
        // Reheater
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient ReH_Kth = 401.69955; // LP superheater outlet temperature
        parameter MetroscopeModelingLibrary.Units.FrictionCoefficient ReH_Kfr_cold = 134.2858; // LP superheater inlet pressure
        // High Pressure Steam Turbine
        parameter MetroscopeModelingLibrary.Units.Cv HPST_CV_Cvmax = 6647.2905; // HP superheater outlet pressure
        parameter MetroscopeModelingLibrary.Units.Cst HPST_Cst = 60400000.0; // HP steam turbine inlet pressure
        parameter MetroscopeModelingLibrary.Units.Yield ST_eta_is = 0.8438316; // Power output
        // Low Pressure Steam Turbine
        parameter MetroscopeModelingLibrary.Units.Cv LPST_CV_Cvmax = 69310.586; // Low pressure superheater outlet pressure
        parameter MetroscopeModelingLibrary.Units.Cst LPST_Cst = 411424.22; // LP steam turbine inlet pressure
        // Condenser
        parameter MetroscopeModelingLibrary.Units.HeatExchangeCoefficient Cond_Kth = 93661.23; // Condensation pressure
        parameter MetroscopeModelingLibrary.Units.VolumeFlowRate Qv_cond_cold = 2.7321725; // Circulating water outlet temperature
        // Exctraction Pump
        parameter Real pump_a3 = 1735.4259; // Exctraction pump outlet pressure
        parameter Real pump_b3 = 0.70563865; // Exctraction pump outlet temperature
        // Recirculation pump
        parameter Real pumpRec_a3 = 870.66187; // Recirculation pump outlet pressure
        parameter Real pumpRec_b3 = 0.6881236; // Recirculation pump outlet temperature
        parameter Real pumpRec_CV_Cvmax = 52.329174; // Recirculation control valve opening

      // Observables of interest

        output MetroscopeModelingLibrary.Units.NegativeMassFlowRate Q_fuel_source; // Controlled by the gas turbine outlet temperature 10.5
        output Real T_flue_gas_sink;
        output Real Q_pumpRec_out; // Controlled by the economizer input temperature
        output Real turbine_compression_rate;

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
      MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HPST_control_valve
        annotation (Placement(transformation(extent={{-203.25,144.738},{-186.75,
                162.677}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-174,148})));
      MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine HPsteamTurbine
        annotation (Placement(transformation(extent={{-160,132},{-126,164}})));
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
      MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source_air(h_out(
            start=0.3e6))
        annotation (Placement(transformation(extent={{-658,-36},{-638,-16}})));
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
      MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber
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
      MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LPsteamTurbine
        annotation (Placement(transformation(extent={{-14,198},{20,230}})));
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
      MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LPST_control_valve
        annotation (Placement(transformation(extent={{-61.25,210.738},{-44.75,
                228.677}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_LPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-28,214})));
      MetroscopeModelingLibrary.WaterSteam.Machines.Pump pumpRec(Q_in_0=1)
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
            origin={176,-26})));
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
    equation

      //--- Air / Flue Gas System ---

        // Air source
          // Quantities definition
          P_source_air_sensor.P_barA = P_source_air;
          T_source_air_sensor.T_degC = T_source_air;
          Q_source_air_sensor.Q = Q_source_air;
          source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

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
          combustionChamber.LHV = LHV;
          combustionChamber.DP = 0.1e5;

        // Gas Turbine
          // Quantities definition
          turbine_T_out_sensor.T_degC = turbine_T_out;
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
          economiser.nominal_hot_side_temperature_rise = 150;
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
          evaporator.S_vaporising = 100;
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
          HPsuperheater1.nominal_hot_side_temperature_rise = 180;
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
          HPsuperheater2.nominal_hot_side_temperature_rise = 180;
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
          Reheater.nominal_hot_side_temperature_rise = 180;
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
            // Parameters
            HPST_control_valve.Opening = HPST_opening;
            HPsteamTurbine.area_nz = 1;
            HPsteamTurbine.eta_nz = 1;
            HPsteamTurbine.eta_is = LPsteamTurbine.eta_is;
            // Calibrated Parameters
            HPST_control_valve.Cvmax = HPST_CV_Cvmax;
            HPsteamTurbine.Cst = HPST_Cst;
            HPsteamTurbine.eta_is = ST_eta_is;

          // Low Pressure Level
            // Quantities definition
            P_LPST_in_sensor.P_barA = P_LPST_in;
            // Parameters
            LPST_control_valve.Opening = LPST_opening;
            LPsteamTurbine.area_nz = 1;
            LPsteamTurbine.eta_nz = 1;
            // Calibrated Parameters
            LPST_control_valve.Cvmax = LPST_CV_Cvmax;
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
        annotation (Line(points={{72,157.934},{74,157.934},{74,158},{78,158},{78,176},{86,176}},
                        color={28,108,200}));
      connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{52,144.067},{52,131},{109,131}},
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
      connect(gasTurbine.C_W_compressor, airCompressor.C_W_in) annotation (Line(
          points={{-414,-10},{-414,16},{-496,16},{-496,-12}},
          color={244,125,35},
          smooth=Smooth.Bezier));
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
      connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,214},
              {52,214},{52,176.778}},    color={28,108,200}));

      connect(P_HPST_out_sensor.C_out, Reheater.C_cold_in) annotation (Line(points={
              {-102,148},{-63,148},{-63,-5}}, color={28,108,200}));
      connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
        annotation (Line(points={{-624,-26},{-618,-26}}, color={95,95,95}));
      connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
        annotation (Line(points={{-606,-26},{-600,-26}}, color={95,95,95}));
      connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
         Line(points={{31.2,167.889},{30,167.889},{30,159},{24,159}},     color={28,
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
      connect(GT_generator.C_in, gasTurbine.C_W_out) annotation (Line(
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
      connect(P_source_air_sensor.C_in, source_air.C_out)
        annotation (Line(points={{-636,-26},{-643,-26}}, color={95,95,95}));
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
      connect(T_flue_gas_sink_sensor.C_in, economiser.C_hot_out) annotation (Line(points={{170,-26},{123.3,-26},{123.3,-26.5}}, color={95,95,95}));
      connect(T_flue_gas_sink_sensor.C_out, P_flue_gas_sink_sensor.C_in) annotation (Line(points={{182,-26},{222,-26},{222,166}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-680,-120},
                {260,280}})),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-680,-120},{260,280}}),
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
              extent={{-10,10},{10,-10}},
              fillColor={255,82,82},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              origin={-570,210},
              rotation=360),
            Text(
              extent={{-552,214},{-466,204}},
              textColor={0,0,0},
              textString="Boundary Conditions",
              horizontalAlignment=TextAlignment.Left),
            Rectangle(
              extent={{-580,180},{-560,160}},
              pattern=LinePattern.None,
              fillColor={0,140,72},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,0}),
            Text(
              extent={{-552,176},{-466,166}},
              textColor={0,0,0},
              textString="Parameters",
              horizontalAlignment=TextAlignment.Left),
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
              extent={{-580,140},{-560,120}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={244,237,30},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-552,138},{-420,122}},
              textColor={0,0,0},
              horizontalAlignment=TextAlignment.Left,
              textString="Observables not used for calibration"),
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
              extent={{168,-18},{184,-34}},
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
              textString="Desuperheater")}));
    end MetroscopiaCCGT_causality_direct;

    model MetroscopiaCCGT_causality_direct_withStartValues
      extends
        MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT.MetroscopiaCCGT_causality_direct(
    AirFilter(
    C_in(
    P(start=100000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=90000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-10000.0),
    DP_0(start=100000.0),
    DP_f(start=-9989.122),
    DP_f_0(start=100000.0),
    DP_z(start=-10.877613),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.04432005),
    P_in(start=100000.0),
    P_in_0(start=100000.0),
    P_out(start=90000.0),
    P_out_0(start=100000.0),
    Pm(start=95000.0),
    Q(start=500.0),
    Q_0(start=100.0),
    Q_in(start=500.0),
    Q_in_0(start=100.0),
    Q_out(start=-500.0),
    Q_out_0(start=-100.0),
    Qm(start=500.0),
    Qv_in(start=428.23355),
    Qv_in_0(start=0.1),
    Qv_out(start=-475.81503),
    Qv_out_0(start=-0.1),
    Qvm(start=450.77216),
    T_0(start=300.0),
    T_in(start=297.15),
    T_in_0(start=300.0),
    T_out(start=297.15),
    T_out_0(start=300.0),
    Tm(start=297.15),
    W(start=0.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.768,0.232,0.0,0.0,0.0}),
    delta_z(start=1.0),
    delta_z_0(start=0.0),
    h(start=299616.5),
    h_in(start=299616.5),
    h_out(start=299616.5),
    hm(start=299616.5),
    rho_in(start=1.1675872),
    rho_out(start=1.0508285),
    rhom(start=1.1092079),
    state_in(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=100000.0)),
    state_out(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=90000.0))),
    Cond_Kth(start=93661.23),
    Eco_Kfr_cold(start=973146.4),
    Eco_Kfr_hot(start=0.018985651),
    Eco_Kth(start=3403.8103),
    Evap_CV_Cvmax(start=539.1173),
    Evap_Kfr_cold(start=0.0),
    Evap_Kth(start=3279.242),
    Evap_controlValve(
    C_in(
    P(start=12250000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=1460173.4)),
    Cv(start=188.69107),
    Cvmax(start=539.1173),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-250000.0),
    DP_0(start=0.0),
    Opening(start=0.35),
    P_in(start=12250000.0),
    P_in_0(start=100000.0),
    P_out(start=12000000.0),
    P_out_0(start=100000.0),
    Pm(start=12125000.0),
    Q(start=48.0),
    Q_0(start=100.0),
    Q_in(start=48.0),
    Q_in_0(start=100.0),
    Q_out(start=-48.0),
    Q_out_0(start=-100.0),
    Qm(start=48.0),
    Qv_in(start=0.07163776),
    Qv_in_0(start=0.1),
    Qv_out(start=-0.071697205),
    Qv_out_0(start=-0.1),
    Qvm(start=0.07166747),
    T_0(start=300.0),
    T_in(start=593.2409),
    T_in_0(start=300.0),
    T_out(start=593.15),
    T_out_0(start=300.0),
    Tm(start=593.19543),
    W(start=0.0),
    h(start=1460173.4),
    h_in(start=1460173.4),
    h_out(start=1460173.4),
    hm(start=1460173.4),
    rho_in(start=670.0377),
    rho_out(start=669.4822),
    rhom(start=669.75995),
    state_in(
    T(start=593.2409),
    d(start=670.0377),
    h(start=1460173.4),
    p(start=12250000.0),
    phase(start=0)),
    state_out(
    T(start=593.15),
    d(start=669.4822),
    h(start=1460173.4),
    p(start=12000000.0),
    phase(start=0))),
    Evap_opening(start=0.35),
    Evap_opening_sensor(
    Opening(start=0.35),
    Opening_pc(start=35.0),
    Opening_pc_0(start=15.0)),
    Evap_x_steam_out(start=1.0),
    Filter_Kfr(start=0.04432005),
    GT_generator(
    C_in(
    W(start=151515150.0)),
    C_out(
    W(start=-150000000.0)),
    W_elec(start=-150000000.0),
    W_mech(start=151515150.0),
    eta(start=0.99)),
    HPSH1_Kfr_cold(start=7030.31),
    HPSH1_Kth(start=1181.761),
    HPSH2_Kfr_cold(start=2538.3271),
    HPSH2_Kth(start=1661.5535),
    HPST_CV_Cvmax(start=6647.2905),
    HPST_Cst(start=60380820.0),
    HPST_control_valve(
    C_in(
    P(start=11400000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11300000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    Cv(start=6647.2905),
    Cvmax(start=6647.2905),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-100000.0),
    DP_0(start=0.0),
    Opening(start=1.0),
    P_in(start=11400000.0),
    P_in_0(start=100000.0),
    P_out(start=11300000.0),
    P_out_0(start=100000.0),
    Pm(start=11350000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_in(start=50.0),
    Q_in_0(start=100.0),
    Q_out(start=-50.0),
    Q_out_0(start=-100.0),
    Qm(start=50.0),
    Qv_in(start=1.5897398),
    Qv_in_0(start=0.1),
    Qv_out(start=-1.6038849),
    Qv_out_0(start=-0.1),
    Qvm(start=1.596781),
    T_0(start=300.0),
    T_in(start=839.65),
    T_in_0(start=300.0),
    T_out(start=839.27313),
    T_out_0(start=300.0),
    Tm(start=839.46155),
    W(start=0.0),
    h(start=3529789.0),
    h_in(start=3529789.0),
    h_out(start=3529789.0),
    hm(start=3529789.0),
    rho_in(start=31.451687),
    rho_out(start=31.174305),
    rhom(start=31.312996),
    state_in(
    T(start=839.65),
    d(start=31.451687),
    h(start=3529789.0),
    p(start=11400000.0),
    phase(start=0)),
    state_out(
    T(start=839.27313),
    d(start=31.174305),
    h(start=3529789.0),
    p(start=11300000.0),
    phase(start=0))),
    HPST_opening(start=1.0),
    HPsteamTurbine(
    C_W_out(
    W(start=-28721590.0)),
    C_in(
    P(start=11300000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=1000000.0),
    Q(start=-50.0),
    h_outflow(start=2955357.2)),
    Cst(start=60380820.0),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-10300000.0),
    DP_0(start=-500000.0),
    P_in(start=11300000.0),
    P_in_0(start=6000000.0),
    P_out(start=1000000.0),
    P_out_0(start=5500000.0),
    Pm(start=6150000.0),
    Q(start=50.0),
    Q_0(start=1500.0),
    Q_in(start=50.0),
    Q_in_0(start=1500.0),
    Q_out(start=-50.0),
    Q_out_0(start=-1500.0),
    Qm(start=50.0),
    Qv_in(start=1.6038849),
    Qv_in_0(start=1.5),
    Qv_out(start=-11.7784),
    Qv_out_0(start=-1.5),
    Qvm(start=2.8233144),
    T_in(start=839.27313),
    T_in_0(start=300.0),
    T_out(start=528.64996),
    T_out_0(start=300.0),
    Tm(start=683.96155),
    W(start=-28721590.0),
    area_nz(start=1.0),
    eta_is(start=0.8438316),
    eta_nz(start=1.0),
    h_in(start=3529789.0),
    h_is(start=2849046.8),
    h_liq_in(start=1462717.5),
    h_liq_out(start=762682.9),
    h_out(start=2955357.2),
    h_real(start=2955357.2),
    h_vap_in(start=2700337.8),
    h_vap_out(start=2777119.5),
    hm(start=3242573.2),
    rho_in(start=31.174305),
    rho_out(start=4.2450585),
    rhom(start=17.709682),
    state_in(
    T(start=839.27313),
    d(start=31.174305),
    h(start=3529789.0),
    p(start=11300000.0),
    phase(start=0)),
    state_is(
    T(start=481.81647),
    d(start=4.7436023),
    h(start=2849046.8),
    p(start=1000000.0),
    phase(start=0)),
    state_out(
    T(start=528.64996),
    d(start=4.2450585),
    h(start=2955357.2),
    p(start=1000000.0),
    phase(start=0)),
    u_out(start=11.7784),
    x_in(start=1.0),
    x_inner(start=1.0),
    xm(start=1.0)),
    HPsuperheater1(
    C_cold_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=11600000.0),
    Q(start=-48.0),
    h_outflow(start=3216404.0)),
    C_hot_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=908245.6)),
    HX(
    Cp_cold(start=5681.045),
    Cp_cold_0(start=75.0),
    Cp_hot(start=1201.6772),
    Cp_hot_0(start=45.0),
    Cr(start=0.4444932),
    Kth(start=1181.761),
    Kth_0(start=5000.0),
    NTU(start=0.43337134),
    QCpMAX(start=613485.56),
    QCpMIN(start=272690.16),
    Q_cold(start=48.0),
    Q_cold_0(start=1500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    S_0(start=100.0),
    T_cold_in(start=597.8341),
    T_cold_in_0(start=413.15),
    T_hot_in(start=884.8291),
    T_hot_in_0(start=473.15),
    W(start=25479424.0),
    W_max(start=78260720.0),
    epsilon(start=0.32557106)),
    Kfr_cold(start=7030.31),
    Kfr_hot(start=0.0),
    Kth(start=1181.761),
    P_cold_in_0(start=350000.0),
    Q_cold(start=48.0),
    Q_cold_0(start=11.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    T_cold_in(start=597.8341),
    T_cold_in_0(start=413.15),
    T_cold_out(start=725.4141),
    T_hot_in(start=884.8291),
    T_hot_out(start=842.38007),
    W(start=25479424.0),
    cold_side(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=3216404.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-250000.0),
    P(start=12000000.0),
    P_0(start=100000.0),
    P_in(start=12000000.0),
    P_in_0(start=350000.0),
    P_out(start=12000000.0),
    P_out_0(start=100000.0),
    Pm(start=12000000.0),
    Q(start=48.0),
    Q_0(start=11.0),
    Q_in(start=48.0),
    Q_in_0(start=11.0),
    Q_out(start=-48.0),
    Q_out_0(start=-11.0),
    Qm(start=48.0),
    Qv_in(start=0.68495256),
    Qv_in_0(start=0.011),
    Qv_out(start=-1.1653867),
    Qv_out_0(start=-0.011),
    Qvm(start=0.8627981),
    T_in(start=597.8341),
    T_in_0(start=413.15),
    T_out(start=725.4141),
    T_out_0(start=300.0),
    Tm(start=661.6241),
    W(start=25479424.0),
    W_input(start=25479424.0),
    h_in(start=2685582.5),
    h_out(start=3216404.0),
    hm(start=2950993.2),
    rho_in(start=70.07785),
    rho_out(start=41.188046),
    rhom(start=55.632946),
    state_in(
    T(start=597.8341),
    d(start=70.07785),
    h(start=2685582.5),
    p(start=12000000.0),
    phase(start=0)),
    state_out(
    T(start=725.4141),
    d(start=41.188046),
    h(start=3216404.0),
    p(start=12000000.0),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11600000.0),
    Q(start=-48.0),
    h_outflow(start=3216404.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-400000.0),
    DP_0(start=100000.0),
    DP_f(start=-400000.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=7030.31),
    P_in(start=12000000.0),
    P_in_0(start=100000.0),
    P_out(start=11600000.0),
    P_out_0(start=100000.0),
    Pm(start=11800000.0),
    Q(start=48.0),
    Q_0(start=11.0),
    Q_in(start=48.0),
    Q_in_0(start=11.0),
    Q_out(start=-48.0),
    Q_out_0(start=-11.0),
    Qm(start=48.0),
    Qv_in(start=1.1653867),
    Qv_in_0(start=0.011),
    Qv_out(start=-1.205996),
    Qv_out_0(start=-0.011),
    Qvm(start=1.1853436),
    T_0(start=300.0),
    T_in(start=725.4141),
    T_in_0(start=413.15),
    T_out(start=723.15),
    T_out_0(start=300.0),
    Tm(start=724.28204),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=3216404.0),
    h_in(start=3216404.0),
    h_out(start=3216404.0),
    hm(start=3216404.0),
    rho_in(start=41.188046),
    rho_out(start=39.801125),
    rhom(start=40.494587),
    state_in(
    T(start=725.4141),
    d(start=41.188046),
    h(start=3216404.0),
    p(start=12000000.0),
    phase(start=0)),
    state_out(
    T(start=723.15),
    d(start=39.801125),
    h(start=3216404.0),
    p(start=11600000.0),
    phase(start=0))),
    hot_side(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=908245.6)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=110000.0),
    P_0(start=100000.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1201.5078),
    Qv_in_0(start=0.05),
    Qv_out(start=-1143.8662),
    Qv_out_0(start=-0.05),
    Qvm(start=1171.9788),
    T_in(start=884.8291),
    T_in_0(start=300.0),
    T_out(start=842.38007),
    T_out_0(start=300.0),
    Tm(start=863.60455),
    W(start=-25479424.0),
    W_input(start=-25479424.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=958153.94),
    h_out(start=908245.6),
    hm(start=933199.8),
    rho_in(start=0.42490312),
    rho_out(start=0.44631478),
    rhom(start=0.43560895),
    state_in(
    T(start=884.8291),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=842.38007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    hot_side_pipe(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=958153.94)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1201.5078),
    Qv_in_0(start=0.05),
    Qv_out(start=-1201.5078),
    Qv_out_0(start=-0.05),
    Qvm(start=1201.5078),
    T_0(start=300.0),
    T_in(start=884.8291),
    T_in_0(start=300.0),
    T_out(start=884.8291),
    T_out_0(start=300.0),
    Tm(start=884.8291),
    W(start=0.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=958153.94),
    h_in(start=958153.94),
    h_out(start=958153.94),
    hm(start=958153.94),
    rho_in(start=0.42490312),
    rho_out(start=0.42490312),
    rhom(start=0.42490312),
    state_in(
    T(start=884.8291),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=884.8291),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    nominal_cold_side_temperature_rise(start=250.0),
    nominal_hot_side_temperature_rise(start=180.0)),
    HPsuperheater2(
    C_cold_in(
    P(start=11600000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=11400000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    C_hot_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=958153.94)),
    HX(
    Cp_cold(start=2880.3398),
    Cp_cold_0(start=75.0),
    Cp_hot(start=1209.5925),
    Cp_hot_0(start=45.0),
    Cr(start=0.23321588),
    Kth(start=1661.5535),
    Kth_0(start=5000.0),
    NTU(start=1.1537205),
    QCpMAX(start=617526.5),
    QCpMIN(start=144017.0),
    Q_cold(start=50.0),
    Q_cold_0(start=1500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    S_0(start=100.0),
    T_cold_in(start=685.6732),
    T_cold_in_0(start=413.15),
    T_hot_in(start=920.1309),
    T_hot_in_0(start=473.15),
    W(start=21363428.0),
    W_max(start=33765890.0),
    epsilon(start=0.63269264)),
    Kfr_cold(start=2538.3271),
    Kfr_hot(start=0.0),
    Kth(start=1661.5535),
    P_cold_in_0(start=350000.0),
    Q_cold(start=50.0),
    Q_cold_0(start=11.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    T_cold_in(start=685.6732),
    T_cold_in_0(start=413.15),
    T_cold_out(start=840.4018),
    T_hot_in(start=920.1309),
    T_hot_out(start=884.8291),
    W(start=21363428.0),
    cold_side(
    C_in(
    P(start=11600000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11600000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-250000.0),
    P(start=11600000.0),
    P_0(start=100000.0),
    P_in(start=11600000.0),
    P_in_0(start=350000.0),
    P_out(start=11600000.0),
    P_out_0(start=100000.0),
    Pm(start=11600000.0),
    Q(start=50.0),
    Q_0(start=11.0),
    Q_in(start=50.0),
    Q_in_0(start=11.0),
    Q_out(start=-50.0),
    Q_out_0(start=-11.0),
    Qm(start=50.0),
    Qv_in(start=1.1427548),
    Qv_in_0(start=0.011),
    Qv_out(start=-1.5621833),
    Qv_out_0(start=-0.011),
    Qvm(start=1.3199507),
    T_in(start=685.6732),
    T_in_0(start=413.15),
    T_out(start=840.4018),
    T_out_0(start=300.0),
    Tm(start=763.03754),
    W(start=21363428.0),
    W_input(start=21363428.0),
    h_in(start=3102520.5),
    h_out(start=3529789.0),
    hm(start=3316154.8),
    rho_in(start=43.753918),
    rho_out(start=32.00649),
    rhom(start=37.880203),
    state_in(
    T(start=685.6732),
    d(start=43.753918),
    h(start=3102520.5),
    p(start=11600000.0),
    phase(start=0)),
    state_out(
    T(start=840.4018),
    d(start=32.00649),
    h(start=3529789.0),
    p(start=11600000.0),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=11600000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11400000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-200000.0),
    DP_0(start=100000.0),
    DP_f(start=-200000.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=2538.3271),
    P_in(start=11600000.0),
    P_in_0(start=100000.0),
    P_out(start=11400000.0),
    P_out_0(start=100000.0),
    Pm(start=11500000.0),
    Q(start=50.0),
    Q_0(start=11.0),
    Q_in(start=50.0),
    Q_in_0(start=11.0),
    Q_out(start=-50.0),
    Q_out_0(start=-11.0),
    Qm(start=50.0),
    Qv_in(start=1.5621833),
    Qv_in_0(start=0.011),
    Qv_out(start=-1.5897398),
    Qv_out_0(start=-0.011),
    Qvm(start=1.5758411),
    T_0(start=300.0),
    T_in(start=840.4018),
    T_in_0(start=413.15),
    T_out(start=839.65),
    T_out_0(start=300.0),
    Tm(start=840.0259),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=3529789.0),
    h_in(start=3529789.0),
    h_out(start=3529789.0),
    hm(start=3529789.0),
    rho_in(start=32.00649),
    rho_out(start=31.451687),
    rhom(start=31.729088),
    state_in(
    T(start=840.4018),
    d(start=32.00649),
    h(start=3529789.0),
    p(start=11600000.0),
    phase(start=0)),
    state_out(
    T(start=839.65),
    d(start=31.451687),
    h(start=3529789.0),
    p(start=11400000.0),
    phase(start=0))),
    hot_side(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=958153.94)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=110000.0),
    P_0(start=100000.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1249.444),
    Qv_in_0(start=0.05),
    Qv_out(start=-1201.5078),
    Qv_out_0(start=-0.05),
    Qvm(start=1225.0071),
    T_in(start=920.1309),
    T_in_0(start=300.0),
    T_out(start=884.8291),
    T_out_0(start=300.0),
    Tm(start=902.48),
    W(start=-21363428.0),
    W_input(start=-21363428.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=1000000.0),
    h_out(start=958153.94),
    hm(start=979077.0),
    rho_in(start=0.40860128),
    rho_out(start=0.42490312),
    rhom(start=0.4167522),
    state_in(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=884.8291),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    hot_side_pipe(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1000000.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1249.444),
    Qv_in_0(start=0.05),
    Qv_out(start=-1249.444),
    Qv_out_0(start=-0.05),
    Qvm(start=1249.444),
    T_0(start=300.0),
    T_in(start=920.1309),
    T_in_0(start=300.0),
    T_out(start=920.1309),
    T_out_0(start=300.0),
    Tm(start=920.1309),
    W(start=0.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=1000000.0),
    h_in(start=1000000.0),
    h_out(start=1000000.0),
    hm(start=1000000.0),
    rho_in(start=0.40860128),
    rho_out(start=0.40860128),
    rhom(start=0.40860128),
    state_in(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    nominal_cold_side_temperature_rise(start=150.0),
    nominal_hot_side_temperature_rise(start=180.0)),
    LHV(start=48130000.0),
    LPST_CV_Cvmax(start=69310.586),
    LPST_Cst(start=411424.22),
    LPST_control_valve(
    C_in(
    P(start=900000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=800000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    Cv(start=69310.586),
    Cvmax(start=69310.586),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-100000.0),
    DP_0(start=0.0),
    Opening(start=1.0),
    P_in(start=900000.0),
    P_in_0(start=100000.0),
    P_out(start=800000.0),
    P_out_0(start=100000.0),
    Pm(start=850000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_in(start=50.0),
    Q_in_0(start=100.0),
    Q_out(start=-50.0),
    Q_out_0(start=-100.0),
    Qm(start=50.0),
    Qv_in(start=15.722852),
    Qv_in_0(start=0.1),
    Qv_out(start=-17.692139),
    Qv_out_0(start=-0.1),
    Qvm(start=16.649466),
    T_0(start=300.0),
    T_in(start=623.15),
    T_in_0(start=300.0),
    T_out(start=622.20447),
    T_out_0(start=300.0),
    Tm(start=622.67725),
    W(start=0.0),
    h(start=3160163.2),
    h_in(start=3160163.2),
    h_out(start=3160163.2),
    hm(start=3160163.2),
    rho_in(start=3.1800847),
    rho_out(start=2.826114),
    rhom(start=3.0030992),
    state_in(
    T(start=623.15),
    d(start=3.1800847),
    h(start=3160163.2),
    p(start=900000.0),
    phase(start=0)),
    state_out(
    T(start=622.20447),
    d(start=2.826114),
    h(start=3160163.2),
    p(start=800000.0),
    phase(start=0))),
    LPST_opening(start=1.0),
    LPsteamTurbine(
    C_W_out(
    W(start=-36934976.0)),
    C_in(
    P(start=800000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=5000.0),
    Q(start=-50.0),
    h_outflow(start=2421463.8)),
    Cst(start=411424.22),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-795000.0),
    DP_0(start=-500000.0),
    P_in(start=800000.0),
    P_in_0(start=6000000.0),
    P_out(start=5000.0),
    P_out_0(start=5500000.0),
    Pm(start=402500.0),
    Q(start=50.0),
    Q_0(start=1500.0),
    Q_in(start=50.0),
    Q_in_0(start=1500.0),
    Q_out(start=-50.0),
    Q_out_0(start=-1500.0),
    Qm(start=50.0),
    Qv_in(start=17.692139),
    Qv_in_0(start=1.5),
    Qv_out(start=-1328.2548),
    Qv_out_0(start=-1.5),
    Qvm(start=34.91916),
    T_in(start=622.20447),
    T_in_0(start=300.0),
    T_out(start=306.01807),
    T_out_0(start=300.0),
    Tm(start=464.11127),
    W(start=-36934976.0),
    area_nz(start=1.0),
    eta_is(start=0.8438316),
    eta_nz(start=1.0),
    h_in(start=3160163.2),
    h_is(start=2258843.2),
    h_liq_in(start=721017.9),
    h_liq_out(start=137765.12),
    h_out(start=2421463.8),
    h_real(start=2421463.8),
    h_vap_in(start=2768302.2),
    h_vap_out(start=2560765.0),
    hm(start=2790813.5),
    rho_in(start=2.826114),
    rho_out(start=0.03764338),
    rhom(start=1.4318787),
    state_in(
    T(start=622.20447),
    d(start=2.826114),
    h(start=3160163.2),
    p(start=800000.0),
    phase(start=0)),
    state_is(
    T(start=306.01938),
    d(start=0.040528756),
    h(start=2258843.2),
    p(start=5000.0),
    phase(start=0)),
    state_out(
    T(start=306.01807),
    d(start=0.03764338),
    h(start=2421463.8),
    p(start=5000.0),
    phase(start=0)),
    u_out(start=1328.2548),
    x_in(start=1.0),
    x_inner(start=0.94250876),
    xm(start=0.9712544)),
    P_Cond(start=0.05),
    P_Cond_sensor(
    C_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=5000.0),
    Q(start=-50.0),
    h_outflow(start=2421463.8)),
    P(start=5000.0),
    P_0(start=100000.0),
    P_barA(start=0.05),
    P_barG(start=-0.95),
    P_mbar(start=50.0),
    P_psi(start=0.72519),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=2421463.8),
    state(
    T(start=306.01807),
    d(start=0.03764338),
    h(start=2421463.8),
    p(start=5000.0),
    phase(start=0))),
    P_HPST_in_sensor(
    C_in(
    P(start=11300000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11300000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    P(start=11300000.0),
    P_0(start=100000.0),
    P_barA(start=113.0),
    P_barG(start=112.0),
    P_mbar(start=113000.0),
    P_psi(start=1638.9294),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=3529789.0),
    state(
    T(start=839.27313),
    d(start=31.174305),
    h(start=3529789.0),
    p(start=11300000.0),
    phase(start=0))),
    P_HPST_out_sensor(
    C_in(
    P(start=1000000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=1000000.0),
    Q(start=-50.0),
    h_outflow(start=2955357.2)),
    P(start=1000000.0),
    P_0(start=100000.0),
    P_barA(start=10.0),
    P_barG(start=9.0),
    P_mbar(start=10000.0),
    P_psi(start=145.038),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=2955357.2),
    state(
    T(start=528.64996),
    d(start=4.2450585),
    h(start=2955357.2),
    p(start=1000000.0),
    phase(start=0))),
    P_LPST_in(start=8.0),
    P_LPST_in_sensor(
    C_in(
    P(start=800000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=800000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    P(start=800000.0),
    P_0(start=100000.0),
    P_barA(start=8.0),
    P_barG(start=7.0),
    P_mbar(start=8000.0),
    P_psi(start=116.0304),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=3160163.2),
    state(
    T(start=622.20447),
    d(start=2.826114),
    h(start=3160163.2),
    p(start=800000.0),
    phase(start=0))),
    P_ST_in(start=113.0),
    P_ST_out(start=10.0),
    P_circulating_water_in_sensor(
    C_in(
    P(start=500000.0),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_out(
    P(start=500000.0),
    Q(start=-2730.2144),
    h_outflow(start=63375.0)),
    P(start=500000.0),
    P_0(start=100000.0),
    P_barA(start=5.0),
    P_barG(start=4.0),
    P_mbar(start=5000.0),
    P_psi(start=72.519),
    Q(start=2730.2144),
    Q_0(start=100.0),
    h(start=63375.0),
    state(
    T(start=288.15),
    d(start=999.287),
    h(start=63375.0),
    p(start=500000.0),
    phase(start=0))),
    P_filter_out(start=0.9),
    P_filter_out_sensor(
    C_in(
    P(start=90000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=90000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    P(start=90000.0),
    P_0(start=100000.0),
    P_barA(start=0.9),
    P_barG(start=-0.1),
    P_mbar(start=900.0),
    P_psi(start=13.05342),
    Q(start=500.0),
    Q_0(start=100.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=299616.5),
    state(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=90000.0))),
    P_flue_gas_sink_sensor(
    C_in(
    P(start=100000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=650319.44)),
    P(start=100000.0),
    P_0(start=100000.0),
    P_barA(start=1.0),
    P_barG(start=0.0),
    P_mbar(start=1000.0),
    P_psi(start=14.5038),
    Q(start=510.5244),
    Q_0(start=100.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=650319.44),
    state(
    T(start=616.1443),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0))),
    P_fuel_source_sensor(
    C_in(
    P(start=3000000.0),
    Q(start=10.524413),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=3000000.0),
    Q(start=-10.524413),
    Xi_outflow(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_outflow(start=899265.0)),
    P(start=3000000.0),
    P_0(start=100000.0),
    P_barA(start=30.0),
    P_barG(start=29.0),
    P_mbar(start=30000.0),
    P_psi(start=435.114),
    Q(start=10.524413),
    Q_0(start=100.0),
    Xi(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h(start=899265.0),
    state(
    T(start=429.15),
    X(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    p(start=3000000.0))),
    P_pumpRec_out(start=180.0),
    P_pumpRec_out_sensor(
    C_in(
    P(start=18000000.0),
    Q(start=9.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=18000000.0),
    Q(start=-9.402336),
    h_outflow(start=1472581.5)),
    P(start=18000000.0),
    P_0(start=100000.0),
    P_barA(start=180.0),
    P_barG(start=179.0),
    P_mbar(start=180000.0),
    P_psi(start=2610.684),
    Q(start=9.402336),
    Q_0(start=100.0),
    h(start=1472581.5),
    state(
    T(start=597.15),
    d(start=676.8383),
    h(start=1472581.5),
    p(start=18000000.0),
    phase(start=0))),
    P_pump_out(start=170.0),
    P_pump_out_sensor(
    C_in(
    P(start=17000000.0),
    Q(start=50.0),
    h_outflow(start=1472581.5)),
    C_out(
    P(start=17000000.0),
    Q(start=-50.0),
    h_outflow(start=161852.23)),
    P(start=17000000.0),
    P_0(start=100000.0),
    P_barA(start=170.0),
    P_barG(start=169.0),
    P_mbar(start=170000.0),
    P_psi(start=2465.646),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=161852.23),
    state(
    T(start=308.15),
    d(start=1001.3589),
    h(start=161852.23),
    p(start=17000000.0),
    phase(start=0))),
    P_source_air_sensor(
    C_in(
    P(start=100000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    P(start=100000.0),
    P_0(start=100000.0),
    P_barA(start=1.0),
    P_barG(start=0.0),
    P_mbar(start=1000.0),
    P_psi(start=14.5038),
    Q(start=500.0),
    Q_0(start=100.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=299616.5),
    state(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=100000.0))),
    P_w_HPSH1_out(start=116.0),
    P_w_HPSH1_out_sensor(
    C_in(
    P(start=11600000.0),
    Q(start=48.0),
    h_outflow(start=369317.44)),
    C_out(
    P(start=11600000.0),
    Q(start=-48.0),
    h_outflow(start=3216404.0)),
    P(start=11600000.0),
    P_0(start=100000.0),
    P_barA(start=116.0),
    P_barG(start=115.0),
    P_mbar(start=116000.0),
    P_psi(start=1682.4408),
    Q(start=48.0),
    Q_0(start=100.0),
    h(start=3216404.0),
    state(
    T(start=723.15),
    d(start=39.801125),
    h(start=3216404.0),
    p(start=11600000.0),
    phase(start=0))),
    P_w_HPSH2_out(start=114.0),
    P_w_HPSH2_out_sensor(
    C_in(
    P(start=11400000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11400000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    P(start=11400000.0),
    P_0(start=100000.0),
    P_barA(start=114.0),
    P_barG(start=113.0),
    P_mbar(start=114000.0),
    P_psi(start=1653.4332),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=3529789.0),
    state(
    T(start=839.65),
    d(start=31.451687),
    h(start=3529789.0),
    p(start=11400000.0),
    phase(start=0))),
    P_w_ReH_out(start=9.0),
    P_w_ReH_out_sensor(
    C_in(
    P(start=900000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=900000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    P(start=900000.0),
    P_0(start=100000.0),
    P_barA(start=9.0),
    P_barG(start=8.0),
    P_mbar(start=9000.0),
    P_psi(start=130.5342),
    Q(start=50.0),
    Q_0(start=100.0),
    h(start=3160163.2),
    state(
    T(start=623.15),
    d(start=3.1800847),
    h(start=3160163.2),
    p(start=900000.0),
    phase(start=0))),
    P_w_eco_out(start=122.5),
    P_w_eco_out_sensor(
    C_in(
    P(start=12250000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12250000.0),
    Q(start=-48.0),
    h_outflow(start=1460173.4)),
    P(start=12250000.0),
    P_0(start=100000.0),
    P_barA(start=122.5),
    P_barG(start=121.5),
    P_mbar(start=122500.0),
    P_psi(start=1776.7155),
    Q(start=48.0),
    Q_0(start=100.0),
    h(start=1460173.4),
    state(
    T(start=593.2409),
    d(start=670.0377),
    h(start=1460173.4),
    p(start=12250000.0),
    phase(start=0))),
    P_w_evap_out(start=120.0),
    P_w_evap_out_sensor(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=2685582.5)),
    P(start=12000000.0),
    P_0(start=100000.0),
    P_barA(start=120.0),
    P_barG(start=119.0),
    P_mbar(start=120000.0),
    P_psi(start=1740.456),
    Q(start=48.0),
    Q_0(start=100.0),
    h(start=2685582.5),
    state(
    T(start=597.8341),
    d(start=70.07785),
    h(start=2685582.5),
    p(start=12000000.0),
    phase(start=0))),
    Q_deSH(start=2.0),
    Q_deSH_sensor(
    C_in(
    P(start=17000000.0),
    Q(start=2.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-2.0),
    h_outflow(start=369317.44)),
    P(start=17000000.0),
    P_0(start=100000.0),
    Q(start=2.0),
    Q_0(start=100.0),
    Q_lbs(start=0.90718484),
    Q_th(start=7.2),
    Qv(start=0.002048979),
    Qv_0(start=0.1),
    h(start=369317.44),
    state(
    T(start=358.15),
    d(start=976.0959),
    h(start=369317.44),
    p(start=17000000.0),
    phase(start=0))),
    Q_fuel_source(start=10.524413),
    Q_fuel_source_sensor(
    C_in(
    P(start=3000000.0),
    Q(start=10.524413),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=3000000.0),
    Q(start=-10.524413),
    Xi_outflow(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_outflow(start=899265.0)),
    P(start=3000000.0),
    P_0(start=100000.0),
    Q(start=10.524413),
    Q_0(start=100.0),
    Q_lbs(start=4.773794),
    Q_th(start=37.887886),
    Qv(start=0.74134684),
    Qv_0(start=0.1),
    Xi(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h(start=899265.0),
    state(
    T(start=429.15),
    X(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    p(start=3000000.0))),
    Q_pumpRec_out(start=9.402336),
    Q_pumpRec_out_sensor(
    C_in(
    P(start=18000000.0),
    Q(start=9.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=18000000.0),
    Q(start=-9.402336),
    h_outflow(start=1472581.5)),
    P(start=18000000.0),
    P_0(start=100000.0),
    Q(start=9.402336),
    Q_0(start=100.0),
    Q_lbs(start=4.264828),
    Q_th(start=33.848408),
    Qv(start=0.0138915535),
    Qv_0(start=0.1),
    h(start=1472581.5),
    state(
    T(start=597.15),
    d(start=676.8383),
    h(start=1472581.5),
    p(start=18000000.0),
    phase(start=0))),
    Q_pump_out(start=50.0),
    Q_pump_out_sensor(
    C_in(
    P(start=17000000.0),
    Q(start=50.0),
    h_outflow(start=1472581.5)),
    C_out(
    P(start=17000000.0),
    Q(start=-50.0),
    h_outflow(start=161852.23)),
    P(start=17000000.0),
    P_0(start=100000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_lbs(start=22.67962),
    Q_th(start=180.0),
    Qv(start=0.04993215),
    Qv_0(start=0.1),
    h(start=161852.23),
    state(
    T(start=308.15),
    d(start=1001.3589),
    h(start=161852.23),
    p(start=17000000.0),
    phase(start=0))),
    Q_source_air_sensor(
    C_in(
    P(start=100000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    P(start=100000.0),
    P_0(start=100000.0),
    Q(start=500.0),
    Q_0(start=100.0),
    Q_lbs(start=226.79622),
    Q_th(start=1800.0),
    Qv(start=428.23355),
    Qv_0(start=0.1),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=299616.5),
    state(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=100000.0))),
    Qv_cond_cold(start=2.7321725),
    ReH_Kfr_cold(start=134.2858),
    ReH_Kth(start=401.69955),
    Reheater(
    C_cold_in(
    P(start=1000000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=900000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    C_hot_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=888187.2)),
    HX(
    Cp_cold(start=2161.1333),
    Cp_cold_0(start=75.0),
    Cp_hot(start=1191.7789),
    Cp_hot_0(start=45.0),
    Cr(start=0.17759852),
    Kth(start=401.69955),
    Kth_0(start=5000.0),
    NTU(start=0.37174898),
    QCpMAX(start=608432.2),
    QCpMIN(start=108056.66),
    Q_cold(start=50.0),
    Q_cold_0(start=1500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    S_0(start=100.0),
    T_cold_in(start=528.64996),
    T_cold_in_0(start=413.15),
    T_hot_in(start=842.38007),
    T_hot_in_0(start=473.15),
    W(start=10240303.0),
    W_max(start=33900628.0),
    epsilon(start=0.30206823)),
    Kfr_cold(start=134.2858),
    Kfr_hot(start=0.0),
    Kth(start=401.69955),
    P_cold_in_0(start=350000.0),
    Q_cold(start=50.0),
    Q_cold_0(start=11.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    T_cold_in(start=528.64996),
    T_cold_in_0(start=413.15),
    T_cold_out(start=624.09125),
    T_hot_in(start=842.38007),
    T_hot_out(start=825.2085),
    W(start=10240303.0),
    cold_side(
    C_in(
    P(start=1000000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=1000000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-250000.0),
    P(start=1000000.0),
    P_0(start=100000.0),
    P_in(start=1000000.0),
    P_in_0(start=350000.0),
    P_out(start=1000000.0),
    P_out_0(start=100000.0),
    Pm(start=1000000.0),
    Q(start=50.0),
    Q_0(start=11.0),
    Q_in(start=50.0),
    Q_in_0(start=11.0),
    Q_out(start=-50.0),
    Q_out_0(start=-11.0),
    Qm(start=50.0),
    Qv_in(start=11.7784),
    Qv_in_0(start=0.011),
    Qv_out(start=-14.147453),
    Qv_out_0(start=-0.011),
    Qvm(start=12.854687),
    T_in(start=528.64996),
    T_in_0(start=413.15),
    T_out(start=624.09125),
    T_out_0(start=300.0),
    Tm(start=576.3706),
    W(start=10240303.0),
    W_input(start=10240303.0),
    h_in(start=2955357.2),
    h_out(start=3160163.2),
    hm(start=3057760.2),
    rho_in(start=4.2450585),
    rho_out(start=3.534205),
    rhom(start=3.889632),
    state_in(
    T(start=528.64996),
    d(start=4.2450585),
    h(start=2955357.2),
    p(start=1000000.0),
    phase(start=0)),
    state_out(
    T(start=624.09125),
    d(start=3.534205),
    h(start=3160163.2),
    p(start=1000000.0),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=1000000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=900000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-100000.0),
    DP_0(start=100000.0),
    DP_f(start=-100000.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=134.2858),
    P_in(start=1000000.0),
    P_in_0(start=100000.0),
    P_out(start=900000.0),
    P_out_0(start=100000.0),
    Pm(start=950000.0),
    Q(start=50.0),
    Q_0(start=11.0),
    Q_in(start=50.0),
    Q_in_0(start=11.0),
    Q_out(start=-50.0),
    Q_out_0(start=-11.0),
    Qm(start=50.0),
    Qv_in(start=14.147453),
    Qv_in_0(start=0.011),
    Qv_out(start=-15.722852),
    Qv_out_0(start=-0.011),
    Qvm(start=14.893608),
    T_0(start=300.0),
    T_in(start=624.09125),
    T_in_0(start=413.15),
    T_out(start=623.15),
    T_out_0(start=300.0),
    Tm(start=623.6206),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=3160163.2),
    h_in(start=3160163.2),
    h_out(start=3160163.2),
    hm(start=3160163.2),
    rho_in(start=3.534205),
    rho_out(start=3.1800847),
    rhom(start=3.3571448),
    state_in(
    T(start=624.09125),
    d(start=3.534205),
    h(start=3160163.2),
    p(start=1000000.0),
    phase(start=0)),
    state_out(
    T(start=623.15),
    d(start=3.1800847),
    h(start=3160163.2),
    p(start=900000.0),
    phase(start=0))),
    hot_side(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=888187.2)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=110000.0),
    P_0(start=100000.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1143.8662),
    Qv_in_0(start=0.05),
    Qv_out(start=-1120.5491),
    Qv_out_0(start=-0.05),
    Qvm(start=1132.0876),
    T_in(start=842.38007),
    T_in_0(start=300.0),
    T_out(start=825.2085),
    T_out_0(start=300.0),
    Tm(start=833.79425),
    W(start=-10240303.0),
    W_input(start=-10240303.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=908245.6),
    h_out(start=888187.2),
    hm(start=898216.44),
    rho_in(start=0.44631478),
    rho_out(start=0.45560202),
    rhom(start=0.4509584),
    state_in(
    T(start=842.38007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=825.2085),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    hot_side_pipe(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=908245.6)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1143.8662),
    Qv_in_0(start=0.05),
    Qv_out(start=-1143.8662),
    Qv_out_0(start=-0.05),
    Qvm(start=1143.8662),
    T_0(start=300.0),
    T_in(start=842.38007),
    T_in_0(start=300.0),
    T_out(start=842.38007),
    T_out_0(start=300.0),
    Tm(start=842.38007),
    W(start=0.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=908245.6),
    h_in(start=908245.6),
    h_out(start=908245.6),
    hm(start=908245.6),
    rho_in(start=0.44631478),
    rho_out(start=0.44631478),
    rhom(start=0.44631478),
    state_in(
    T(start=842.38007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=842.38007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    nominal_cold_side_temperature_rise(start=100.0),
    nominal_hot_side_temperature_rise(start=180.0)),
    ST_eta_is(start=0.8438316),
    ST_generator(
    C_in(
    W(start=65656564.0)),
    C_out(
    W(start=-65000000.0)),
    W_elec(start=-65000000.0),
    W_mech(start=65656564.0),
    eta(start=0.99)),
    T_circulating_water_in_sensor(
    C_in(
    P(start=500000.0),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_out(
    P(start=500000.0),
    Q(start=-2730.2144),
    h_outflow(start=63375.0)),
    P(start=500000.0),
    P_0(start=100000.0),
    Q(start=2730.2144),
    Q_0(start=100.0),
    T(start=288.15),
    T_0(start=300.0),
    T_degC(start=15.0),
    T_degF(start=59.0),
    h(start=63375.0),
    state(
    T(start=288.15),
    d(start=999.287),
    h(start=63375.0),
    p(start=500000.0),
    phase(start=0))),
    T_circulating_water_out(start=25.0),
    T_circulating_water_out_sensor(
    C_in(
    P(start=492540.6),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_out(
    P(start=492540.6),
    Q(start=-2730.2144),
    h_outflow(start=105198.266)),
    P(start=492540.6),
    P_0(start=100000.0),
    Q(start=2730.2144),
    Q_0(start=100.0),
    T(start=298.15),
    T_0(start=300.0),
    T_degC(start=25.0),
    T_degF(start=77.0),
    h(start=105198.266),
    state(
    T(start=298.15),
    d(start=997.2241),
    h(start=105198.266),
    p(start=492540.6),
    phase(start=0))),
    T_flue_gas_sink(start=342.9943),
    T_flue_gas_sink_sensor(
    C_in(
    P(start=100000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=650319.44)),
    P(start=100000.0),
    P_0(start=100000.0),
    Q(start=510.5244),
    Q_0(start=100.0),
    T(start=616.1443),
    T_0(start=300.0),
    T_degC(start=342.9943),
    T_degF(start=649.3897),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=650319.44),
    state(
    T(start=616.1443),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0))),
    T_fuel_source_sensor(
    C_in(
    P(start=3000000.0),
    Q(start=10.524413),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=3000000.0),
    Q(start=-10.524413),
    Xi_outflow(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_outflow(start=899265.0)),
    P(start=3000000.0),
    P_0(start=100000.0),
    Q(start=10.524413),
    Q_0(start=100.0),
    T(start=429.15),
    T_0(start=300.0),
    T_degC(start=156.0),
    T_degF(start=312.8),
    Xi(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h(start=899265.0),
    state(
    T(start=429.15),
    X(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    p(start=3000000.0))),
    T_pumpRec_out(start=324.0),
    T_pumpRec_out_sensor(
    C_in(
    P(start=18000000.0),
    Q(start=9.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=18000000.0),
    Q(start=-9.402336),
    h_outflow(start=1472581.5)),
    P(start=18000000.0),
    P_0(start=100000.0),
    Q(start=9.402336),
    Q_0(start=100.0),
    T(start=597.15),
    T_0(start=300.0),
    T_degC(start=324.0),
    T_degF(start=615.2),
    h(start=1472581.5),
    state(
    T(start=597.15),
    d(start=676.8383),
    h(start=1472581.5),
    p(start=18000000.0),
    phase(start=0))),
    T_pump_out(start=35.0),
    T_pump_out_sensor(
    C_in(
    P(start=17000000.0),
    Q(start=50.0),
    h_outflow(start=1472581.5)),
    C_out(
    P(start=17000000.0),
    Q(start=-50.0),
    h_outflow(start=161852.23)),
    P(start=17000000.0),
    P_0(start=100000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    T(start=308.15),
    T_0(start=300.0),
    T_degC(start=35.0),
    T_degF(start=95.0),
    h(start=161852.23),
    state(
    T(start=308.15),
    d(start=1001.3589),
    h(start=161852.23),
    p(start=17000000.0),
    phase(start=0))),
    T_source_air_sensor(
    C_in(
    P(start=100000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    P(start=100000.0),
    P_0(start=100000.0),
    Q(start=500.0),
    Q_0(start=100.0),
    T(start=297.15),
    T_0(start=300.0),
    T_degC(start=24.0),
    T_degF(start=75.2),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=299616.5),
    state(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=100000.0))),
    T_w_HPSH1_out(start=450.0),
    T_w_HPSH1_out_sensor(
    C_in(
    P(start=11600000.0),
    Q(start=48.0),
    h_outflow(start=369317.44)),
    C_out(
    P(start=11600000.0),
    Q(start=-48.0),
    h_outflow(start=3216404.0)),
    P(start=11600000.0),
    P_0(start=100000.0),
    Q(start=48.0),
    Q_0(start=100.0),
    T(start=723.15),
    T_0(start=300.0),
    T_degC(start=450.0),
    T_degF(start=842.0),
    h(start=3216404.0),
    state(
    T(start=723.15),
    d(start=39.801125),
    h(start=3216404.0),
    p(start=11600000.0),
    phase(start=0))),
    T_w_HPSH2_out(start=566.5),
    T_w_HPSH2_out_sensor(
    C_in(
    P(start=11400000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11400000.0),
    Q(start=-50.0),
    h_outflow(start=3529789.0)),
    P(start=11400000.0),
    P_0(start=100000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    T(start=839.65),
    T_0(start=300.0),
    T_degC(start=566.5),
    T_degF(start=1051.7),
    h(start=3529789.0),
    state(
    T(start=839.65),
    d(start=31.451687),
    h(start=3529789.0),
    p(start=11400000.0),
    phase(start=0))),
    T_w_ReH_out(start=350.0),
    T_w_ReH_out_sensor(
    C_in(
    P(start=900000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=900000.0),
    Q(start=-50.0),
    h_outflow(start=3160163.2)),
    P(start=900000.0),
    P_0(start=100000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    T(start=623.15),
    T_0(start=300.0),
    T_degC(start=350.0),
    T_degF(start=662.0),
    h(start=3160163.2),
    state(
    T(start=623.15),
    d(start=3.1800847),
    h(start=3160163.2),
    p(start=900000.0),
    phase(start=0))),
    T_w_eco_in(start=85.0),
    T_w_eco_in_sensor(
    C_in(
    P(start=17000000.0),
    Q(start=57.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-57.402336),
    h_outflow(start=369317.44)),
    P(start=17000000.0),
    P_0(start=100000.0),
    Q(start=57.402336),
    Q_0(start=100.0),
    T(start=358.15),
    T_0(start=300.0),
    T_degC(start=85.0),
    T_degF(start=185.0),
    h(start=369317.44),
    state(
    T(start=358.15),
    d(start=976.0959),
    h(start=369317.44),
    p(start=17000000.0),
    phase(start=0))),
    T_w_eco_out(start=320.0),
    T_w_eco_out_sensor(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=1460173.4)),
    P(start=12000000.0),
    P_0(start=100000.0),
    Q(start=48.0),
    Q_0(start=100.0),
    T(start=593.15),
    T_0(start=300.0),
    T_degC(start=320.0),
    T_degF(start=608.0),
    h(start=1460173.4),
    state(
    T(start=593.15),
    d(start=669.4822),
    h(start=1460173.4),
    p(start=12000000.0),
    phase(start=0))),
    W_GT(start=150.0),
    W_GT_sensor(
    C_in(
    W(start=150000000.0)),
    C_out(
    W(start=-150000000.0)),
    W(start=150000000.0),
    W_MW(start=150.0)),
    W_ST_out(start=65.0),
    W_ST_out_sensor(
    C_in(
    W(start=65000000.0)),
    C_out(
    W(start=-65000000.0)),
    W(start=65000000.0),
    W_MW(start=65.0)),
    airCompressor(
    C_W_in(
    W(start=222002940.0)),
    C_in(
    P(start=90000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=1700000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=743622.4)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=1610000.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P_in(start=90000.0),
    P_in_0(start=100000.0),
    P_out(start=1700000.0),
    P_out_0(start=100000.0),
    Pm(start=895000.0),
    Q(start=500.0),
    Q_0(start=100.0),
    Q_in(start=500.0),
    Q_in_0(start=100.0),
    Q_out(start=-500.0),
    Q_out_0(start=-100.0),
    Qm(start=500.0),
    Qv_in(start=475.81503),
    Qv_in_0(start=0.1),
    Qv_out(start=-61.30338),
    Qv_out_0(start=-0.1),
    Qvm(start=108.61318),
    T_in(start=297.15),
    T_in_0(start=300.0),
    T_out(start=723.15),
    T_out_0(start=300.0),
    Tm(start=510.15),
    W(start=222002940.0),
    Wmech(start=222002940.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.768,0.232,0.0,0.0,0.0}),
    eta_is(start=0.88200104),
    h_in(start=299616.5),
    h_is(start=691230.2),
    h_out(start=743622.4),
    hm(start=521619.44),
    rho_in(start=1.0508285),
    rho_out(start=8.1561575),
    rhom(start=4.6034927),
    state_in(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=90000.0)),
    state_is(
    T(start=674.7305),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=1700000.0)),
    state_out(
    T(start=723.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=1700000.0)),
    tau(start=18.88889)),
    circulating_water_sink(
    C_in(
    P(start=492540.6),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    P_in(start=492540.6),
    Q_in(start=2730.2144),
    Qv_in(start=2.7378142),
    T_in(start=298.15),
    h_in(start=105198.266),
    state_in(
    T(start=298.15),
    d(start=997.2241),
    h(start=105198.266),
    p(start=492540.6),
    phase(start=0))),
    circulating_water_source(
    C_out(
    P(start=500000.0),
    Q(start=-2730.2144),
    h_outflow(start=63375.0)),
    P_out(start=500000.0),
    Q_out(start=-2730.2144),
    Qv_out(start=-2.7321622),
    T_out(start=288.15),
    h_out(start=63375.0),
    state_out(
    T(start=288.15),
    d(start=999.287),
    h(start=63375.0),
    p(start=500000.0),
    phase(start=0))),
    combustionChamber(
    DP(start=10000.0),
    LHV(start=48130000.0),
    Q_air(start=500.0),
    Q_exhaust(start=510.5244),
    Q_fuel(start=10.524413),
    Wth(start=506540000.0),
    X_fuel_C(start=0.7205717),
    X_fuel_C2H6(start=0.05),
    X_fuel_C3H8(start=0.0),
    X_fuel_C4H10_n_butane(start=0.0),
    X_fuel_CH4(start=0.9),
    X_fuel_CO2(start=0.025),
    X_fuel_H(start=0.23625103),
    X_fuel_N2(start=0.025),
    X_fuel_O(start=0.018177252),
    X_in_CO2(start=0.0),
    X_in_H2O(start=0.0),
    X_in_N2(start=0.768),
    X_in_O2(start=0.232),
    X_in_SO2(start=0.0),
    X_out_CO2(start=0.054430123),
    X_out_H2O(start=0.043522727),
    X_out_N2(start=0.7526831),
    X_out_O2(start=0.14936402),
    X_out_SO2(start=0.0),
    h_exhaust(start=1739026.4),
    h_in_air(start=743622.4),
    h_in_air_0(start=500000.0),
    h_in_fuel(start=899265.0),
    inlet(
    P(start=1700000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    inlet1(
    P(start=3000000.0),
    Q(start=10.524413),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    m_C(start=12.0106),
    m_C2H6(start=30.06908),
    m_C3H8(start=45.10362),
    m_C4H10(start=58.1222),
    m_CH4(start=16.04252),
    m_CO2(start=44.0094),
    m_H(start=1.00798),
    m_H2O(start=18.01536),
    m_O(start=15.9994),
    outlet(
    P(start=1690000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1739026.4)),
    sink_air(
    C_in(
    P(start=1700000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    P_in(start=1700000.0),
    Q_in(start=500.0),
    Qv_in(start=61.30338),
    T_in(start=723.15),
    Xi_in(start={0.768,0.232,0.0,0.0,0.0}),
    h_in(start=743622.4),
    state_in(
    T(start=723.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=1700000.0))),
    sink_fuel(
    C_in(
    P(start=3000000.0),
    Q(start=10.524413),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    P_in(start=3000000.0),
    Q_in(start=10.524413),
    Qv_in(start=0.74134684),
    T_in(start=429.15),
    Xi_in(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_in(start=899265.0),
    state_in(
    T(start=429.15),
    X(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    p(start=3000000.0))),
    source_exhaust(
    C_out(
    P(start=1690000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1739026.4)),
    P_out(start=1690000.0),
    Q_out(start=-510.5244),
    Qv_out(start=-133.65562),
    T_out(start=1512.2168),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_out(start=1739026.4),
    state_out(
    T(start=1512.2168),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=1690000.0)))),
    compression_rate(start=18.88889),
    compressor_P_out(start=17.0),
    compressor_P_out_sensor(
    C_in(
    P(start=1700000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=1700000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=743622.4)),
    P(start=1700000.0),
    P_0(start=100000.0),
    P_barA(start=17.0),
    P_barG(start=16.0),
    P_mbar(start=17000.0),
    P_psi(start=246.5646),
    Q(start=500.0),
    Q_0(start=100.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=743622.4),
    state(
    T(start=723.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=1700000.0))),
    compressor_T_out(start=450.0),
    compressor_T_out_sensor(
    C_in(
    P(start=1700000.0),
    Q(start=500.0),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=1700000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=743622.4)),
    P(start=1700000.0),
    P_0(start=100000.0),
    Q(start=500.0),
    Q_0(start=100.0),
    T(start=723.15),
    T_0(start=300.0),
    T_degC(start=450.0),
    T_degF(start=842.0),
    Xi(start={0.768,0.232,0.0,0.0,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=743622.4),
    state(
    T(start=723.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=1700000.0))),
    compressor_eta_is(start=0.88200104),
    condenser(
    C_cold_in(
    P(start=500000.0),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=492540.6),
    Q(start=-2730.2144),
    h_outflow(start=105198.266)),
    C_hot_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=14754.728),
    Q(start=-50.0),
    h_outflow(start=137734.06)),
    C_incond(start=0.0),
    Kfr_cold(start=1.0),
    Kth(start=93661.23),
    P_incond(start=0.0),
    P_offset(start=0.0),
    P_tot(start=5000.0),
    Psat(start=5000.0),
    Psat_0(start=19000.0),
    Q_cold(start=2730.2144),
    Q_cold_0(start=3820.0),
    Q_hot(start=50.0),
    Q_hot_0(start=150.0),
    Qv_cold_in(start=2.7321725),
    R(start=8.314463),
    S(start=100.0),
    T_cold_in(start=288.1517),
    T_cold_out(start=298.15),
    T_hot_in(start=306.01807),
    T_hot_out(start=306.03815),
    Tsat(start=306.01807),
    W(start=114186490.0),
    air_intake(start=0.0),
    cold_side(
    C_in(
    P(start=492540.6),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_out(
    P(start=492540.6),
    Q(start=-2730.2144),
    h_outflow(start=105198.266)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    P(start=492540.6),
    P_0(start=100000.0),
    P_in(start=492540.6),
    P_in_0(start=100000.0),
    P_out(start=492540.6),
    P_out_0(start=100000.0),
    Pm(start=492540.6),
    Q(start=2730.2144),
    Q_0(start=3820.0),
    Q_in(start=2730.2144),
    Q_in_0(start=3820.0),
    Q_out(start=-2730.2144),
    Q_out_0(start=-3820.0),
    Qm(start=2730.2144),
    Qv_in(start=2.7321725),
    Qv_in_0(start=3.82),
    Qv_out(start=-2.7378142),
    Qv_out_0(start=-3.82),
    Qvm(start=2.7349906),
    T_in(start=288.1517),
    T_in_0(start=300.0),
    T_out(start=298.15),
    T_out_0(start=300.0),
    Tm(start=293.15085),
    W(start=114186490.0),
    W_input(start=114186490.0),
    h_in(start=63375.0),
    h_out(start=105198.266),
    hm(start=84286.63),
    rho_in(start=999.28326),
    rho_out(start=997.2241),
    rhom(start=998.25366),
    state_in(
    T(start=288.1517),
    d(start=999.28326),
    h(start=63375.0),
    p(start=492540.6),
    phase(start=0)),
    state_out(
    T(start=298.15),
    d(start=997.2241),
    h(start=105198.266),
    p(start=492540.6),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=500000.0),
    Q(start=2730.2144),
    h_outflow(start=0.0)),
    C_out(
    P(start=492540.6),
    Q(start=-2730.2144),
    h_outflow(start=63375.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-7459.403),
    DP_0(start=100000.0),
    DP_f(start=-7459.403),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=1.0),
    P_in(start=500000.0),
    P_in_0(start=100000.0),
    P_out(start=492540.6),
    P_out_0(start=100000.0),
    Pm(start=496270.3),
    Q(start=2730.2144),
    Q_0(start=3820.0),
    Q_in(start=2730.2144),
    Q_in_0(start=3820.0),
    Q_out(start=-2730.2144),
    Q_out_0(start=-3820.0),
    Qm(start=2730.2144),
    Qv_in(start=2.7321622),
    Qv_in_0(start=3.82),
    Qv_out(start=-2.7321725),
    Qv_out_0(start=-3.82),
    Qvm(start=2.7321675),
    T_0(start=300.0),
    T_in(start=288.15),
    T_in_0(start=300.0),
    T_out(start=288.1517),
    T_out_0(start=300.0),
    Tm(start=288.15085),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=63375.0),
    h_in(start=63375.0),
    h_out(start=63375.0),
    hm(start=63375.0),
    rho_in(start=999.287),
    rho_out(start=999.28326),
    rhom(start=999.28516),
    state_in(
    T(start=288.15),
    d(start=999.287),
    h(start=63375.0),
    p(start=500000.0),
    phase(start=0)),
    state_out(
    T(start=288.1517),
    d(start=999.28326),
    h(start=63375.0),
    p(start=492540.6),
    phase(start=0))),
    faulty(start=false),
    fouling(start=0.0),
    hot_side(
    C_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=5000.0),
    Q(start=-50.0),
    h_outflow(start=137734.06)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    P(start=5000.0),
    P_0(start=19000.0),
    P_in(start=5000.0),
    P_in_0(start=19000.0),
    P_out(start=5000.0),
    P_out_0(start=19000.0),
    Pm(start=5000.0),
    Q(start=50.0),
    Q_0(start=150.0),
    Q_in(start=50.0),
    Q_in_0(start=150.0),
    Q_out(start=-50.0),
    Q_out_0(start=-150.0),
    Qm(start=50.0),
    Qv_in(start=1328.2548),
    Qv_in_0(start=0.15),
    Qv_out(start=-0.05026627),
    Qv_out_0(start=-0.15),
    Qvm(start=0.10052873),
    T_in(start=306.01807),
    T_in_0(start=300.0),
    T_out(start=306.03815),
    T_out_0(start=300.0),
    Tm(start=306.0281),
    W(start=-114186490.0),
    W_input(start=-114186490.0),
    h_in(start=2421463.8),
    h_out(start=137734.06),
    hm(start=1279598.9),
    rho_in(start=0.03764338),
    rho_out(start=994.7028),
    rhom(start=497.37024),
    state_in(
    T(start=306.01807),
    d(start=0.03764338),
    h(start=2421463.8),
    p(start=5000.0),
    phase(start=0)),
    state_out(
    T(start=306.03815),
    d(start=994.7028),
    h(start=137734.06),
    p(start=5000.0),
    phase(start=0))),
    incondensables_in(
    C_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=5000.0),
    Q(start=-50.0),
    h_outflow(start=2421463.8)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DP_input(start=0.0),
    P_in(start=5000.0),
    P_in_0(start=100000.0),
    P_out(start=5000.0),
    P_out_0(start=100000.0),
    Pm(start=5000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_in(start=50.0),
    Q_in_0(start=100.0),
    Q_out(start=-50.0),
    Q_out_0(start=-100.0),
    Qm(start=50.0),
    Qv_in(start=1328.2548),
    Qv_in_0(start=0.1),
    Qv_out(start=-1328.2548),
    Qv_out_0(start=-0.1),
    Qvm(start=1328.2548),
    T_0(start=300.0),
    T_in(start=306.01807),
    T_in_0(start=300.0),
    T_out(start=306.01807),
    T_out_0(start=300.0),
    Tm(start=306.01807),
    W(start=0.0),
    h(start=2421463.8),
    h_in(start=2421463.8),
    h_out(start=2421463.8),
    hm(start=2421463.8),
    rho_in(start=0.03764338),
    rho_out(start=0.03764338),
    rhom(start=0.03764338),
    state_in(
    T(start=306.01807),
    d(start=0.03764338),
    h(start=2421463.8),
    p(start=5000.0),
    phase(start=0)),
    state_out(
    T(start=306.01807),
    d(start=0.03764338),
    h(start=2421463.8),
    p(start=5000.0),
    phase(start=0))),
    incondensables_out(
    C_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=5000.0),
    Q(start=-50.0),
    h_outflow(start=137734.06)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DP_input(start=0.0),
    P_in(start=5000.0),
    P_in_0(start=100000.0),
    P_out(start=5000.0),
    P_out_0(start=100000.0),
    Pm(start=5000.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_in(start=50.0),
    Q_in_0(start=100.0),
    Q_out(start=-50.0),
    Q_out_0(start=-100.0),
    Qm(start=50.0),
    Qv_in(start=0.05026627),
    Qv_in_0(start=0.1),
    Qv_out(start=-0.05026627),
    Qv_out_0(start=-0.1),
    Qvm(start=0.05026627),
    T_0(start=300.0),
    T_in(start=306.03815),
    T_in_0(start=300.0),
    T_out(start=306.03815),
    T_out_0(start=300.0),
    Tm(start=306.03815),
    W(start=0.0),
    h(start=137734.06),
    h_in(start=137734.06),
    h_out(start=137734.06),
    hm(start=137734.06),
    rho_in(start=994.7028),
    rho_out(start=994.7028),
    rhom(start=994.7028),
    state_in(
    T(start=306.03815),
    d(start=994.7028),
    h(start=137734.06),
    p(start=5000.0),
    phase(start=0)),
    state_out(
    T(start=306.03815),
    d(start=994.7028),
    h(start=137734.06),
    p(start=5000.0),
    phase(start=0))),
    water_height(start=1.0),
    water_height_pipe(
    C_in(
    P(start=5000.0),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=14754.728),
    Q(start=-50.0),
    h_outflow(start=137734.06)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=9754.728),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=9754.728),
    DP_z_0(start=0.0),
    Kfr(start=0.0),
    P_in(start=5000.0),
    P_in_0(start=19000.0),
    P_out(start=14754.728),
    P_out_0(start=19000.0),
    Pm(start=9877.363),
    Q(start=50.0),
    Q_0(start=150.0),
    Q_in(start=50.0),
    Q_in_0(start=150.0),
    Q_out(start=-50.0),
    Q_out_0(start=-150.0),
    Qm(start=50.0),
    Qv_in(start=0.05026627),
    Qv_in_0(start=0.15),
    Qv_out(start=-0.050266016),
    Qv_out_0(start=-0.15),
    Qvm(start=0.050266143),
    T_0(start=300.0),
    T_in(start=306.03815),
    T_in_0(start=300.0),
    T_out(start=306.03604),
    T_out_0(start=300.0),
    Tm(start=306.03708),
    W(start=0.0),
    delta_z(start=-1.0),
    delta_z_0(start=0.0),
    h(start=137734.06),
    h_in(start=137734.06),
    h_out(start=137734.06),
    hm(start=137734.06),
    rho_in(start=994.7028),
    rho_out(start=994.7078),
    rhom(start=994.7053),
    state_in(
    T(start=306.03815),
    d(start=994.7028),
    h(start=137734.06),
    p(start=5000.0),
    phase(start=0)),
    state_out(
    T(start=306.03604),
    d(start=994.7078),
    h(start=137734.06),
    p(start=14754.728),
    phase(start=0)))),
    deSH_CV_Cvmax(start=7.7502966),
    deSH_controlValve(
    C_in(
    P(start=17000000.0),
    Q(start=2.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=11600000.0),
    Q(start=-2.0),
    h_outflow(start=369317.44)),
    Cv(start=1.1625445),
    Cvmax(start=7.7502966),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-5400000.0),
    DP_0(start=0.0),
    Opening(start=0.15),
    P_in(start=17000000.0),
    P_in_0(start=100000.0),
    P_out(start=11600000.0),
    P_out_0(start=100000.0),
    Pm(start=14300000.0),
    Q(start=2.0),
    Q_0(start=100.0),
    Q_in(start=2.0),
    Q_in_0(start=100.0),
    Q_out(start=-2.0),
    Q_out_0(start=-100.0),
    Qm(start=2.0),
    Qv_in(start=0.002048979),
    Qv_in_0(start=0.1),
    Qv_out(start=-0.0020553127),
    Qv_out_0(start=-0.1),
    Qvm(start=0.002052141),
    T_0(start=300.0),
    T_in(start=358.15),
    T_in_0(start=300.0),
    T_out(start=359.16998),
    T_out_0(start=300.0),
    Tm(start=358.66),
    W(start=0.0),
    h(start=369317.44),
    h_in(start=369317.44),
    h_out(start=369317.44),
    hm(start=369317.44),
    rho_in(start=976.0959),
    rho_out(start=973.08795),
    rhom(start=974.5919),
    state_in(
    T(start=358.15),
    d(start=976.0959),
    h(start=369317.44),
    p(start=17000000.0),
    phase(start=0)),
    state_out(
    T(start=359.16998),
    d(start=973.08795),
    h(start=369317.44),
    p(start=11600000.0),
    phase(start=0))),
    deSH_opening(start=0.15),
    deSH_opening_sensor(
    Opening(start=0.15),
    Opening_pc(start=15.0),
    Opening_pc_0(start=15.0)),
    economiser(
    C_cold_in(
    P(start=17000000.0),
    Q(start=57.402336),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=12250000.0),
    Q(start=-57.402336),
    h_outflow(start=1460173.4)),
    C_hot_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=100000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=650319.44)),
    HX(
    Cp_cold(start=5099.8374),
    Cp_cold_0(start=75.0),
    Cp_hot(start=1158.8867),
    Cp_hot_0(start=45.0),
    Cr(start=0.49479854),
    Kth(start=3403.8103),
    Kth_0(start=5000.0),
    NTU(start=1.1627315),
    QCpMAX(start=591639.94),
    QCpMIN(start=292742.56),
    Q_cold(start=57.402336),
    Q_cold_0(start=1500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    S_0(start=100.0),
    T_cold_in(start=358.15),
    T_cold_in_0(start=349.15),
    T_hot_in(start=725.2495),
    T_hot_in_0(start=473.15),
    W(start=62617684.0),
    W_max(start=107465650.0),
    epsilon(start=0.5826763)),
    Kfr_cold(start=973146.4),
    Kfr_hot(start=0.018985651),
    Kth(start=3403.8103),
    P_cold_in_0(start=1800000.0),
    Q_cold(start=57.402336),
    Q_cold_0(start=178.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    T_cold_in(start=358.15),
    T_cold_in_0(start=349.15),
    T_cold_out(start=594.8165),
    T_hot_in(start=725.2495),
    T_hot_out(start=616.1443),
    W(start=62617684.0),
    cold_side(
    C_in(
    P(start=17000000.0),
    Q(start=57.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-57.402336),
    h_outflow(start=1460173.4)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-1700000.0),
    P(start=17000000.0),
    P_0(start=100000.0),
    P_in(start=17000000.0),
    P_in_0(start=1800000.0),
    P_out(start=17000000.0),
    P_out_0(start=100000.0),
    Pm(start=17000000.0),
    Q(start=57.402336),
    Q_0(start=178.0),
    Q_in(start=57.402336),
    Q_in_0(start=178.0),
    Q_out(start=-57.402336),
    Q_out_0(start=-178.0),
    Qm(start=57.402336),
    Qv_in(start=0.05880809),
    Qv_in_0(start=0.178),
    Qv_out(start=-0.084404476),
    Qv_out_0(start=-0.178),
    Qvm(start=0.06931886),
    T_in(start=358.15),
    T_in_0(start=349.15),
    T_out(start=594.8165),
    T_out_0(start=300.0),
    Tm(start=476.48328),
    W(start=62617684.0),
    W_input(start=62617684.0),
    h_in(start=369317.44),
    h_out(start=1460173.4),
    hm(start=914745.44),
    rho_in(start=976.0959),
    rho_out(start=680.08636),
    rhom(start=828.0911),
    state_in(
    T(start=358.15),
    d(start=976.0959),
    h(start=369317.44),
    p(start=17000000.0),
    phase(start=0)),
    state_out(
    T(start=594.8165),
    d(start=680.08636),
    h(start=1460173.4),
    p(start=17000000.0),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=17000000.0),
    Q(start=57.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=12250000.0),
    Q(start=-57.402336),
    h_outflow(start=1460173.4)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-4750000.0),
    DP_0(start=100000.0),
    DP_f(start=-4750000.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=973146.4),
    P_in(start=17000000.0),
    P_in_0(start=100000.0),
    P_out(start=12250000.0),
    P_out_0(start=100000.0),
    Pm(start=14625000.0),
    Q(start=57.402336),
    Q_0(start=178.0),
    Q_in(start=57.402336),
    Q_in_0(start=178.0),
    Q_out(start=-57.402336),
    Q_out_0(start=-178.0),
    Qm(start=57.402336),
    Qv_in(start=0.084404476),
    Qv_in_0(start=0.178),
    Qv_out(start=-0.0856703),
    Qv_out_0(start=-0.178),
    Qvm(start=0.08503268),
    T_0(start=300.0),
    T_in(start=594.8165),
    T_in_0(start=349.15),
    T_out(start=593.2409),
    T_out_0(start=300.0),
    Tm(start=594.02875),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=1460173.4),
    h_in(start=1460173.4),
    h_out(start=1460173.4),
    hm(start=1460173.4),
    rho_in(start=680.08636),
    rho_out(start=670.0377),
    rhom(start=675.0621),
    state_in(
    T(start=594.8165),
    d(start=680.08636),
    h(start=1460173.4),
    p(start=17000000.0),
    phase(start=0)),
    state_out(
    T(start=593.2409),
    d(start=670.0377),
    h(start=1460173.4),
    p(start=12250000.0),
    phase(start=0))),
    hot_side(
    C_in(
    P(start=100000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=650319.44)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=100000.0),
    P_0(start=100000.0),
    P_in(start=100000.0),
    P_in_0(start=100000.0),
    P_out(start=100000.0),
    P_out_0(start=100000.0),
    Pm(start=100000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1083.2964),
    Qv_in_0(start=0.05),
    Qv_out(start=-920.3273),
    Qv_out_0(start=-0.05),
    Qvm(start=995.1841),
    T_in(start=725.2495),
    T_in_0(start=300.0),
    T_out(start=616.1443),
    T_out_0(start=300.0),
    Tm(start=670.6969),
    W(start=-62617684.0),
    W_input(start=-62617684.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=772973.06),
    h_out(start=650319.44),
    hm(start=711646.25),
    rho_in(start=0.47126937),
    rho_out(start=0.5547205),
    rhom(start=0.51299495),
    state_in(
    T(start=725.2495),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0)),
    state_out(
    T(start=616.1443),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0))),
    hot_side_pipe(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=100000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=772973.06)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-10000.0),
    DP_0(start=100000.0),
    DP_f(start=-10000.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.018985651),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=100000.0),
    P_out_0(start=100000.0),
    Pm(start=105000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=984.8149),
    Qv_in_0(start=0.05),
    Qv_out(start=-1083.2964),
    Qv_out_0(start=-0.05),
    Qvm(start=1031.7108),
    T_0(start=300.0),
    T_in(start=725.2495),
    T_in_0(start=300.0),
    T_out(start=725.2495),
    T_out_0(start=300.0),
    Tm(start=725.2495),
    W(start=0.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=772973.06),
    h_in(start=772973.06),
    h_out(start=772973.06),
    hm(start=772973.06),
    rho_in(start=0.5183963),
    rho_out(start=0.47126937),
    rhom(start=0.49483284),
    state_in(
    T(start=725.2495),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=725.2495),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0))),
    nominal_cold_side_temperature_rise(start=235.0),
    nominal_hot_side_temperature_rise(start=150.0)),
    evaporator(
    C_cold_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_cold_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=2685582.5)),
    C_hot_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_hot_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=772973.06)),
    HX_vaporising(
    Cp_cold(start=0.0),
    Cp_cold_0(start=75.0),
    Cp_hot(start=1165.8937),
    Cp_hot_0(start=45.0),
    Cr(start=5.952172e-05),
    Kth(start=3279.242),
    Kth_0(start=5000.0),
    NTU(start=0.550932),
    QCpMAX(start=10000000000.0),
    QCpMIN(start=595217.2),
    Q_cold(start=48.0),
    Q_cold_0(start=1500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S(start=100.0),
    S_0(start=100.0),
    T_cold_in(start=597.8455),
    T_cold_in_0(start=349.15),
    T_hot_in(start=825.2085),
    T_hot_in_0(start=473.15),
    W(start=57324268.0),
    W_max(start=135330350.0),
    epsilon(start=0.42358765)),
    Kfr_cold(start=0.0),
    Kfr_hot(start=0.0),
    Kth(start=3279.242),
    P_cold_in_0(start=1800000.0),
    Q_cold(start=48.0),
    Q_cold_0(start=500.0),
    Q_hot(start=510.5244),
    Q_hot_0(start=50.0),
    S_vaporising(start=100.0),
    T_cold_in(start=593.15),
    T_cold_in_0(start=349.15),
    T_cold_out(start=597.8455),
    T_hot_in(start=825.2085),
    T_hot_out(start=727.82007),
    Tsat(start=597.8455),
    W_heating(start=1495373.8),
    W_vaporising(start=57324268.0),
    cold_side_heating(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=1491327.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-1700000.0),
    P(start=12000000.0),
    P_0(start=100000.0),
    P_in(start=12000000.0),
    P_in_0(start=1800000.0),
    P_out(start=12000000.0),
    P_out_0(start=100000.0),
    Pm(start=12000000.0),
    Q(start=48.0),
    Q_0(start=500.0),
    Q_in(start=48.0),
    Q_in_0(start=500.0),
    Q_out(start=-48.0),
    Q_out_0(start=-500.0),
    Qm(start=48.0),
    Qv_in(start=0.071697205),
    Qv_in_0(start=0.5),
    Qv_out(start=-0.07327),
    Qv_out_0(start=-0.5),
    Qvm(start=0.072475076),
    T_in(start=593.15),
    T_in_0(start=349.15),
    T_out(start=597.8455),
    T_out_0(start=300.0),
    Tm(start=595.49774),
    W(start=1495373.8),
    W_input(start=1495373.8),
    h_in(start=1460173.4),
    h_out(start=1491327.0),
    hm(start=1475750.2),
    rho_in(start=669.4822),
    rho_out(start=655.1112),
    rhom(start=662.2967),
    state_in(
    T(start=593.15),
    d(start=669.4822),
    h(start=1460173.4),
    p(start=12000000.0),
    phase(start=0)),
    state_out(
    T(start=597.8455),
    d(start=655.1112),
    h(start=1491327.0),
    p(start=12000000.0),
    phase(start=0))),
    cold_side_pipe(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=1460173.4)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    Kfr(start=0.0),
    P_in(start=12000000.0),
    P_in_0(start=100000.0),
    P_out(start=12000000.0),
    P_out_0(start=100000.0),
    Pm(start=12000000.0),
    Q(start=48.0),
    Q_0(start=500.0),
    Q_in(start=48.0),
    Q_in_0(start=500.0),
    Q_out(start=-48.0),
    Q_out_0(start=-500.0),
    Qm(start=48.0),
    Qv_in(start=0.071697205),
    Qv_in_0(start=0.5),
    Qv_out(start=-0.071697205),
    Qv_out_0(start=-0.5),
    Qvm(start=0.071697205),
    T_0(start=300.0),
    T_in(start=593.15),
    T_in_0(start=349.15),
    T_out(start=593.15),
    T_out_0(start=300.0),
    Tm(start=593.15),
    W(start=0.0),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=1460173.4),
    h_in(start=1460173.4),
    h_out(start=1460173.4),
    hm(start=1460173.4),
    rho_in(start=669.4822),
    rho_out(start=669.4822),
    rhom(start=669.4822),
    state_in(
    T(start=593.15),
    d(start=669.4822),
    h(start=1460173.4),
    p(start=12000000.0),
    phase(start=0)),
    state_out(
    T(start=593.15),
    d(start=669.4822),
    h(start=1460173.4),
    p(start=12000000.0),
    phase(start=0))),
    cold_side_vaporising(
    C_in(
    P(start=12000000.0),
    Q(start=48.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=12000000.0),
    Q(start=-48.0),
    h_outflow(start=2685582.5)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=-1700000.0),
    P(start=12000000.0),
    P_0(start=100000.0),
    P_in(start=12000000.0),
    P_in_0(start=1800000.0),
    P_out(start=12000000.0),
    P_out_0(start=100000.0),
    Pm(start=12000000.0),
    Q(start=48.0),
    Q_0(start=500.0),
    Q_in(start=48.0),
    Q_in_0(start=500.0),
    Q_out(start=-48.0),
    Q_out_0(start=-500.0),
    Qm(start=48.0),
    Qv_in(start=0.07327),
    Qv_in_0(start=0.5),
    Qv_out(start=-0.68495256),
    Qv_out_0(start=-0.5),
    Qvm(start=0.13237928),
    T_in(start=597.8455),
    T_in_0(start=349.15),
    T_out(start=597.8341),
    T_out_0(start=300.0),
    Tm(start=597.8398),
    W(start=57324268.0),
    W_input(start=57324268.0),
    h_in(start=1491327.0),
    h_out(start=2685582.5),
    hm(start=2088454.9),
    rho_in(start=655.1112),
    rho_out(start=70.07785),
    rhom(start=362.59454),
    state_in(
    T(start=597.8455),
    d(start=655.1112),
    h(start=1491327.0),
    p(start=12000000.0),
    phase(start=0)),
    state_out(
    T(start=597.8341),
    d(start=70.07785),
    h(start=2685582.5),
    p(start=12000000.0),
    phase(start=0))),
    h_liq_sat(start=1491327.0),
    h_vap_sat(start=2685582.5),
    hot_side_heating(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=772973.06)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=110000.0),
    P_0(start=100000.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=988.3055),
    Qv_in_0(start=0.05),
    Qv_out(start=-984.8149),
    Qv_out_0(start=-0.05),
    Qvm(start=986.5571),
    T_in(start=727.82007),
    T_in_0(start=300.0),
    T_out(start=725.2495),
    T_out_0(start=300.0),
    Tm(start=726.5348),
    W(start=-1495373.8),
    W_input(start=-1495373.8),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=775902.1),
    h_out(start=772973.06),
    hm(start=774437.6),
    rho_in(start=0.5165654),
    rho_out(start=0.5183963),
    rhom(start=0.51748085),
    state_in(
    T(start=727.82007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=725.2495),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    hot_side_pipe(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=888187.2)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=100000.0),
    DP_f(start=0.0),
    DP_f_0(start=100000.0),
    DP_z(start=0.0),
    DP_z_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    Kfr(start=0.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1120.5491),
    Qv_in_0(start=0.05),
    Qv_out(start=-1120.5491),
    Qv_out_0(start=-0.05),
    Qvm(start=1120.5491),
    T_0(start=300.0),
    T_in(start=825.2085),
    T_in_0(start=300.0),
    T_out(start=825.2085),
    T_out_0(start=300.0),
    Tm(start=825.2085),
    W(start=0.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    delta_z(start=0.0),
    delta_z_0(start=0.0),
    h(start=888187.2),
    h_in(start=888187.2),
    h_out(start=888187.2),
    hm(start=888187.2),
    rho_in(start=0.45560202),
    rho_out(start=0.45560202),
    rhom(start=0.45560202),
    state_in(
    T(start=825.2085),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=825.2085),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    hot_side_vaporising(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=775902.1)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=0.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P(start=110000.0),
    P_0(start=100000.0),
    P_in(start=110000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=110000.0),
    Q(start=510.5244),
    Q_0(start=50.0),
    Q_in(start=510.5244),
    Q_in_0(start=50.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-50.0),
    Qm(start=510.5244),
    Qv_in(start=1120.5491),
    Qv_in_0(start=0.05),
    Qv_out(start=-988.3055),
    Qv_out_0(start=-0.05),
    Qvm(start=1050.2809),
    T_in(start=825.2085),
    T_in_0(start=300.0),
    T_out(start=727.82007),
    T_out_0(start=300.0),
    Tm(start=776.5143),
    W(start=-57324268.0),
    W_input(start=-57324268.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.7481,0.1392,0.0525,0.0601,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=888187.2),
    h_out(start=775902.1),
    hm(start=832044.7),
    rho_in(start=0.45560202),
    rho_out(start=0.5165654),
    rhom(start=0.4860837),
    state_in(
    T(start=825.2085),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=727.82007),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    x_steam_out(start=1.0)),
    flue_gas_sink(
    C_in(
    P(start=100000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    P_in(start=100000.0),
    Q_in(start=510.5244),
    Qv_in(start=920.3273),
    T_in(start=616.1443),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_in(start=650319.44),
    state_in(
    T(start=616.1443),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=100000.0))),
    gasTurbine(
    C_W_compressor(
    W(start=-222002940.0)),
    C_W_out(
    W(start=-151515150.0)),
    C_in(
    P(start=1690000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1000000.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-1580000.0),
    DP_0(start=0.0),
    DXi(start={0.0,0.0,0.0,0.0,0.0}),
    P_in(start=1690000.0),
    P_in_0(start=100000.0),
    P_out(start=110000.0),
    P_out_0(start=100000.0),
    Pm(start=900000.0),
    Q(start=510.5244),
    Q_0(start=100.0),
    Q_in(start=510.5244),
    Q_in_0(start=100.0),
    Q_out(start=-510.5244),
    Q_out_0(start=-100.0),
    Qm(start=510.5244),
    Qv_in(start=133.65562),
    Qv_in_0(start=0.1),
    Qv_out(start=-1249.444),
    Qv_out_0(start=-0.1),
    Qvm(start=241.47966),
    T_in(start=1512.2168),
    T_in_0(start=300.0),
    T_out(start=920.1309),
    T_out_0(start=300.0),
    Tm(start=1216.1738),
    W(start=-377291000.0),
    Wcompressor(start=222002940.0),
    Wmech(start=151515150.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_in(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_in_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xi_out(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_out_0(start={0.0,0.0,0.0,0.0,0.0}),
    Xim(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    eta_is(start=0.8269205),
    eta_mech(start=0.99),
    h_in(start=1739026.4),
    h_is(start=845317.3),
    h_out(start=1000000.0),
    hm(start=1369513.2),
    rho_in(start=3.8197002),
    rho_out(start=0.40860128),
    rhom(start=2.1141508),
    state_in(
    T(start=1512.2168),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=1690000.0)),
    state_is(
    T(start=788.28503),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    state_out(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0)),
    tau(start=15.363636)),
    loopBreaker(
    C_in(
    P(start=17000000.0),
    Q(start=57.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-57.402336),
    h_outflow(start=369317.44)),
    loop_flow_error(start=0.0)),
    powerSource(
    C_out(
    W(start=-1418716.0)),
    W_out(start=-1418716.0)),
    pump(
    C_in(
    P(start=14754.728),
    Q(start=50.0),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-50.0),
    h_outflow(start=161852.23)),
    C_power(
    W(start=1418716.0)),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=16985246.0),
    DP_0(start=0.0),
    P_in(start=14754.728),
    P_in_0(start=100000.0),
    P_out(start=17000000.0),
    P_out_0(start=100000.0),
    Pm(start=8507377.0),
    Q(start=50.0),
    Q_0(start=100.0),
    Q_in(start=50.0),
    Q_in_0(start=100.0),
    Q_out(start=-50.0),
    Q_out_0(start=-100.0),
    Qm(start=50.0),
    Qv_in(start=0.050266016),
    Qv_in_0(start=0.1),
    Qv_out(start=-0.04993215),
    Qv_out_0(start=-0.1),
    Qvm(start=0.050098523),
    R(start=1.0),
    T_in(start=306.03604),
    T_in_0(start=300.0),
    T_out(start=308.15),
    T_out_0(start=300.0),
    Tm(start=307.09302),
    VRot(start=1400.0),
    VRotn(start=1400.0),
    W(start=1205908.6),
    Wh(start=1209940.2),
    Wm(start=1418716.0),
    a1(start=0.0),
    a2(start=0.0),
    a3(start=1735.4259),
    b1(start=0.0),
    b2(start=0.0),
    b3(start=0.70563865),
    h_in(start=137734.06),
    h_out(start=161852.23),
    hm(start=149793.16),
    hn(start=1735.4259),
    rh(start=0.70563865),
    rhmin(start=1e-04),
    rho_in(start=994.7078),
    rho_out(start=1001.3589),
    rhom(start=998.0334),
    rm(start=0.85),
    state_in(
    T(start=306.03604),
    d(start=994.7078),
    h(start=137734.06),
    p(start=14754.728),
    phase(start=0)),
    state_out(
    T(start=308.15),
    d(start=1001.3589),
    h(start=161852.23),
    p(start=17000000.0),
    phase(start=0))),
    pumpRec(
    C_in(
    P(start=12250000.0),
    Q(start=9.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=18000000.0),
    Q(start=-9.402336),
    h_outflow(start=1472581.5)),
    C_power(
    W(start=137252.61)),
    DM(start=0.0),
    DM_0(start=99.0),
    DP(start=5750000.0),
    DP_0(start=0.0),
    P_in(start=12250000.0),
    P_in_0(start=100000.0),
    P_out(start=18000000.0),
    P_out_0(start=100000.0),
    Pm(start=15125000.0),
    Q(start=9.402336),
    Q_0(start=100.0),
    Q_in(start=9.402336),
    Q_in_0(start=1.0),
    Q_out(start=-9.402336),
    Q_out_0(start=-100.0),
    Qm(start=9.402336),
    Qv_in(start=0.014032546),
    Qv_in_0(start=0.001),
    Qv_out(start=-0.0138915535),
    Qv_out_0(start=-0.1),
    Qvm(start=0.013961694),
    R(start=1.0),
    T_in(start=593.2409),
    T_in_0(start=300.0),
    T_out(start=597.15),
    T_out_0(start=300.0),
    Tm(start=595.19543),
    VRot(start=1400.0),
    VRotn(start=1400.0),
    W(start=116664.72),
    Wh(start=117256.766),
    Wm(start=137252.61),
    a1(start=0.0),
    a2(start=0.0),
    a3(start=870.66187),
    b1(start=0.0),
    b2(start=0.0),
    b3(start=0.6881236),
    h_in(start=1460173.4),
    h_out(start=1472581.5),
    hm(start=1466377.5),
    hn(start=870.66187),
    rh(start=0.6881236),
    rhmin(start=1e-04),
    rho_in(start=670.0377),
    rho_out(start=676.8383),
    rhom(start=673.43805),
    rm(start=0.85),
    state_in(
    T(start=593.2409),
    d(start=670.0377),
    h(start=1460173.4),
    p(start=12250000.0),
    phase(start=0)),
    state_out(
    T(start=597.15),
    d(start=676.8383),
    h(start=1472581.5),
    p(start=18000000.0),
    phase(start=0))),
    pumpRec_CV_Cvmax(start=52.329174),
    pumpRec_a3(start=870.66187),
    pumpRec_b3(start=0.6881236),
    pumpRec_controlValve(
    C_in(
    P(start=18000000.0),
    Q(start=9.402336),
    h_outflow(start=0.0)),
    C_out(
    P(start=17000000.0),
    Q(start=-9.402336),
    h_outflow(start=1472581.5)),
    Cv(start=18.31521),
    Cvmax(start=52.329174),
    DM(start=0.0),
    DM_0(start=0.0),
    DP(start=-1000000.0),
    DP_0(start=0.0),
    Opening(start=0.35),
    P_in(start=18000000.0),
    P_in_0(start=100000.0),
    P_out(start=17000000.0),
    P_out_0(start=100000.0),
    Pm(start=17500000.0),
    Q(start=9.402336),
    Q_0(start=100.0),
    Q_in(start=9.402336),
    Q_in_0(start=100.0),
    Q_out(start=-9.402336),
    Q_out_0(start=-100.0),
    Qm(start=9.402336),
    Qv_in(start=0.0138915535),
    Qv_in_0(start=0.1),
    Qv_out(start=-0.01393399),
    Qv_out_0(start=-0.1),
    Qvm(start=0.013912739),
    T_0(start=300.0),
    T_in(start=597.15),
    T_in_0(start=300.0),
    T_out(start=596.8314),
    T_out_0(start=300.0),
    Tm(start=596.9907),
    W(start=0.0),
    h(start=1472581.5),
    h_in(start=1472581.5),
    h_out(start=1472581.5),
    hm(start=1472581.5),
    rho_in(start=676.8383),
    rho_out(start=674.777),
    rhom(start=675.8077),
    state_in(
    T(start=597.15),
    d(start=676.8383),
    h(start=1472581.5),
    p(start=18000000.0),
    phase(start=0)),
    state_out(
    T(start=596.8314),
    d(start=674.777),
    h(start=1472581.5),
    p(start=17000000.0),
    phase(start=0))),
    pumpRec_opening(start=0.35),
    pumpRec_opening_sensor(
    Opening(start=0.35),
    Opening_pc(start=35.0),
    Opening_pc_0(start=15.0)),
    pumpRec_powerSource(
    C_out(
    W(start=-137252.61)),
    W_out(start=-137252.61)),
    pump_a3(start=1735.4259),
    pump_b3(start=0.70563865),
    sink(
    C_in(
    W(start=65000000.0)),
    W_in(start=65000000.0)),
    sink_power(
    C_in(
    W(start=150000000.0)),
    W_in(start=150000000.0)),
    source_air(
    C_out(
    P(start=100000.0),
    Q(start=-500.0),
    Xi_outflow(start={0.768,0.232,0.0,0.0,0.0}),
    h_outflow(start=299616.5)),
    P_out(start=100000.0),
    Q_out(start=-500.0),
    Qv_out(start=-428.23355),
    T_out(start=297.15),
    Xi_out(start={0.768,0.232,0.0,0.0,0.0}),
    h_out(start=299616.5),
    state_out(
    T(start=297.15),
    X(start={0.768,0.232,0.0,0.0,0.0}),
    p(start=100000.0))),
    source_fuel(
    C_out(
    P(start=3000000.0),
    Q(start=-10.524413),
    Xi_outflow(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_outflow(start=899265.0)),
    P_out(start=3000000.0),
    Q_out(start=-10.524413),
    Qv_out(start=-0.74134684),
    T_out(start=429.15),
    Xi_out(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    h_out(start=899265.0),
    state_out(
    T(start=429.15),
    X(start={0.9,0.05,0.0,0.0,0.025,0.025}),
    p(start=3000000.0))),
    turbine_P_out(start=1.1),
    turbine_P_out_sensor(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1000000.0)),
    P(start=110000.0),
    P_0(start=100000.0),
    P_barA(start=1.1),
    P_barG(start=0.1),
    P_mbar(start=1100.0),
    P_psi(start=15.95418),
    Q(start=510.5244),
    Q_0(start=100.0),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=1000000.0),
    state(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    turbine_T_out_sensor(
    C_in(
    P(start=110000.0),
    Q(start=510.5244),
    Xi_outflow(start={0.0,0.0,0.0,0.0,0.0}),
    h_outflow(start=0.0)),
    C_out(
    P(start=110000.0),
    Q(start=-510.5244),
    Xi_outflow(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    h_outflow(start=1000000.0)),
    P(start=110000.0),
    P_0(start=100000.0),
    Q(start=510.5244),
    Q_0(start=100.0),
    T(start=920.1309),
    T_0(start=300.0),
    T_degC(start=646.9809),
    T_degF(start=1196.5657),
    Xi(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    Xi_0(start={0.0,0.0,0.0,0.0,0.0}),
    h(start=1000000.0),
    state(
    T(start=920.1309),
    X(start={0.7526831,0.14936402,0.043522727,0.054430123,0.0}),
    p(start=110000.0))),
    turbine_compression_rate(start=15.363636),
    turbine_eta_is(start=0.8269205));
      annotation (experiment(__Dymola_fixedstepsize=0.1, __Dymola_Algorithm="Euler"));
    end MetroscopiaCCGT_causality_direct_withStartValues;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
  end MetroscopiaCCGT;
  extends Modelica.Icons.ExamplesPackage;
  model GasTurbine_direct

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
    input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

    input Units.Pressure P_fuel(start = 30e5);
    input Units.SpecificEnthalpy h_fuel(start=0.9e6);
    input Units.NegativeMassFlowRate Q_fuel(start=15);

    // Parameters
    parameter Units.SpecificEnthalpy LHV = 48130e3;
    parameter Units.DifferentialPressure combustion_chamber_pressure_loss = 0.1e5;
    parameter Real compression_rate = 17;
    parameter Real compressor_eta_is = 0.9;
    parameter Real turbine_compression_rate = 17;
    parameter Real turbine_eta_is = 0.9;
    parameter Real eta_mech = 0.99;


    FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
    FlueGases.Machines.AirCompressor                           airCompressor annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{66,-10},{86,10}})));
    FlueGases.Machines.GasTurbine                              gasTurbine    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{66,30},{86,50}})));
    MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    Fuel.BoundaryConditions.Source                           source_fuel annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-38})));
  equation

    // Boundary Conditions
    source_air.P_out = source_P;
    source_air.h_out = source_h;
    source_air.Q_out = source_Q;
    source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

    source_fuel.P_out = P_fuel;
    source_fuel.h_out = h_fuel;
    source_fuel.Q_out = - Q_fuel;
    source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Parameters
    combustionChamber.LHV = LHV;
    combustionChamber.DP = combustion_chamber_pressure_loss;
    airCompressor.tau = compression_rate;
    airCompressor.eta_is = compressor_eta_is;
    gasTurbine.tau = turbine_compression_rate;
    gasTurbine.eta_is = turbine_eta_is;
    gasTurbine.eta_mech = eta_mech;



    connect(source_air.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-54,0}}, color={95,95,95}));
    connect(gasTurbine.C_out, sink_exhaust.C_in) annotation (Line(points={{50,0},{71,0}}, color={95,95,95}));
    connect(gasTurbine.C_W_out,sink_power. C_in) annotation (Line(
        points={{50,10},{50,10},{50,40},{71,40}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(airCompressor.C_W_in, gasTurbine.C_W_compressor) annotation (Line(
        points={{-34,10},{-34,26},{30,26},{30,10}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(combustionChamber.inlet1,source_fuel. C_out) annotation (Line(points={{0,-10},{0,-33}},                   color={213,213,0}));
    connect(combustionChamber.outlet, gasTurbine.C_in) annotation (Line(points={{10,0},{30,0}}, color={95,95,95}));
    connect(combustionChamber.inlet, airCompressor.C_out) annotation (Line(points={{-10,0},{-34,0}}, color={95,95,95}));
      annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2})),
      Window(
        x=0.03,
        y=0.02,
        width=0.95,
        height=0.95),
      Icon(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics={Polygon(
            points={{100,100},{100,-100},{-100,-100},{-100,100},{100,100}},
            lineColor={0,0,255},
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-32,62},{34,46},{-10,30},{46,18},{-6,0},{36,-16},{-10,-32},{42,
                -44},{-32,-60},{-32,62}},
            lineColor={0,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.CrossDiag)}));
  end GasTurbine_direct;

  model GasTurbine_reverse

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
    input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

    input Units.Pressure P_fuel(start = 30e5);
    input Units.SpecificEnthalpy h_fuel(start=0.9e6);
    input Units.NegativeMassFlowRate Q_fuel(start=15);

    // Parameters
    parameter Units.SpecificEnthalpy LHV = 48130e3;
    parameter Units.DifferentialPressure combustion_chamber_pressure_loss = 0.1e5;
    parameter Real eta_mech = 0.99;

    // Inputs for calibration
    input Real compressor_P_out(start = 16) "barA";
    input Real compressor_T_out(start = 406) "degC";
    input Real W(start = 200) "MW";
    input Real turbine_P_out(start=1) "barA";

    // Parameters for calibration
    output Real compression_rate;
    output Real compressor_eta_is;
    output Real turbine_compression_rate;
    output Real turbine_eta_is;

    // Initialisation parameters
    parameter Units.SpecificEnthalpy h_out_compressor_0 = 7e5; // Model won't initialize correctly without a guess value for the outlet enthalpy

    FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
    FlueGases.Machines.AirCompressor                           airCompressor(h_out(start=h_out_compressor_0)) annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
    FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{88,-10},{108,10}})));
    FlueGases.Machines.GasTurbine                              gasTurbine(eta_is(
          start=0.73), eta_mech(start=0.9))                                  annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{88,30},{108,50}})));
    MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    Fuel.BoundaryConditions.Source                           source_fuel annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-38})));
    Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
    Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
    Sensors.FlueGases.PressureSensor turbine_P_out_sensor annotation (Placement(transformation(extent={{62,-10},{82,10}})));
    Sensors.Power.PowerSensor W_sensor annotation (Placement(transformation(extent={{64,30},{84,50}})));
  equation

    // Boundary Conditions
    source_air.P_out = source_P;
    source_air.h_out = source_h;
    source_air.Q_out = source_Q;
    source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

    source_fuel.P_out = P_fuel;
    source_fuel.h_out = h_fuel;
    source_fuel.Q_out = - Q_fuel;
    source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Parameters
    combustionChamber.LHV = LHV;
    combustionChamber.DP = combustion_chamber_pressure_loss;
    gasTurbine.eta_mech = eta_mech;

    // Inputs for calibration
    compressor_T_out_sensor.T_degC = compressor_T_out;
    compressor_P_out_sensor.P_barA = compressor_P_out;
    W_sensor.W_MW = W;
    turbine_P_out_sensor.P_barA = turbine_P_out;



    // Parameters for calibration
    airCompressor.tau = compression_rate;
    airCompressor.eta_is = compressor_eta_is;
    gasTurbine.tau = turbine_compression_rate;
    gasTurbine.eta_is = turbine_eta_is;


    connect(source_air.C_out, airCompressor.C_in) annotation (Line(points={{-93,0},{-84,0}}, color={95,95,95}));
    connect(airCompressor.C_W_in, gasTurbine.C_W_compressor) annotation (Line(
        points={{-64,10},{-64,22},{30,22},{30,10}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(combustionChamber.inlet1,source_fuel. C_out) annotation (Line(points={{0,-10},{0,-33}},                   color={213,213,0}));
    connect(combustionChamber.outlet, gasTurbine.C_in) annotation (Line(points={{10,0},{30,0}}, color={95,95,95}));
    connect(sink_power.C_in, W_sensor.C_out) annotation (Line(points={{93,40},{83.8,40}}, color={244,125,35}));
    connect(W_sensor.C_in, gasTurbine.C_W_out) annotation (Line(
        points={{64,40},{50,40},{50,10}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(turbine_P_out_sensor.C_out, sink_exhaust.C_in) annotation (Line(points={{82,0},{93,0}}, color={95,95,95}));
    connect(turbine_P_out_sensor.C_in, gasTurbine.C_out) annotation (Line(points={{62,0},{50,0}}, color={95,95,95}));
    connect(airCompressor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{-64,0},{-58,0}}, color={95,95,95}));
    connect(compressor_P_out_sensor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-38,0},{-34,0}}, color={95,95,95}));
    connect(compressor_T_out_sensor.C_out, combustionChamber.inlet) annotation (Line(points={{-14,0},{-10,0}}, color={95,95,95}));
      annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2})),
      Window(
        x=0.03,
        y=0.02,
        width=0.95,
        height=0.95),
      Icon(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics={Polygon(
            points={{100,100},{100,-100},{-100,-100},{-100,100},{100,100}},
            lineColor={0,0,255},
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-32,62},{34,46},{-10,30},{46,18},{-6,0},{36,-16},{-10,-32},{42,
                -44},{-32,-60},{-32,62}},
            lineColor={0,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.CrossDiag)}));
  end GasTurbine_reverse;

  model Evaporator_noRecirculation_direct
                // Boundary conditions
    input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
    input Units.MassFlowRate Q_hot_source(start=586);
    input Real hot_source_h(start=494000);

    input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
    input Units.MassFlowRate Q_cold_source(start=96);
    input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

     // Parameters
    parameter Units.Area S = 10;
    parameter Units.HeatExchangeCoefficient Kth= 102000;
    parameter Units.FrictionCoefficient Kfr_hot = 0;
    parameter Units.FrictionCoefficient Kfr_cold = 1;


    MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
    WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
    WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
    FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
    FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  equation

      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source;
    hot_source.h_out = hot_source_h;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source;
    cold_source.T_out =  T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    evaporator.S_vaporising = S;
    evaporator.Kth = Kth;
    evaporator.Kfr_hot = Kfr_hot;
    evaporator.Kfr_cold = Kfr_cold;

    connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, cold_steam_sink.C_in) annotation (Line(points={{-46,52},{-85,52}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, cold_liquid_sink.C_in) annotation (Line(points={{-46,44},{-76,44},{-76,-52},{-83,-52}}, color={28,108,200}));
    connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
    connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{28.3,-0.72},{45.65,-0.72},{45.65,0},{59,0}}, color={95,95,95}));
    connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,76},{-4,40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-36,56},{-34,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,58},{-28,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,52},{-30,50}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
            Rectangle(
            extent={{-60,40},{78,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_noRecirculation_direct;

  model Evaporator_noRecirculation_reverse
      input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
    input Units.MassFlowRate Q_hot_source(start=586);
    input Real hot_source_h(start=494000);

    input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
    input Units.MassFlowRate Q_cold_source(start=96);
    input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

    // Parameters
    parameter Units.Area S = 10;


    // Calibrated parameters
    output Units.HeatExchangeCoefficient Kth;
    output Units.FrictionCoefficient Kfr_hot;
    output Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
    input Real P_cold_out(start = 3.5, min=1.5, nominal=3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
    input Real P_hot_out(start=1, min=1, nominal=1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
    input Real Q_cold_liq_out(start = 97, min = 80, nominal = 97) "kg/s"; // Outlet temperature on cold side, to calibrate Kth
    MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
    WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
    WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
    FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
    FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
    Sensors.WaterSteam.PressureSensor                                P_cold_out_sensor annotation (Placement(transformation(extent={{-58,60},{-74,44}})));
    Sensors.WaterSteam.FlowSensor                                Q_cold_liquid_out annotation (Placement(transformation(extent={{8,-8},{-8,8}},
          rotation=90,
          origin={-50,-36})));
    Sensors.FlueGases.PressureSensor                                    P_hot_out_sensor  annotation (Placement(transformation(extent={{44,-4},{52,4}})));
  equation

      // Boundary conditions
    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source;
    hot_source.h_out = hot_source_h;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source;
    cold_source.T_out =  T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    // Parameters
    evaporator.S_vaporising = S;

    // Inputs for calibration
    Q_cold_liquid_out.Q = Q_cold_liq_out;
    P_cold_out_sensor.P_barA = P_cold_out;
    P_hot_out_sensor.P_barA = P_hot_out;

    // Calibrated parameters
    evaporator.Kth = Kth;
    evaporator.Kfr_hot = Kfr_hot;
    evaporator.Kfr_cold = Kfr_cold;

    connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
    connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
    connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
    connect(evaporator.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{28.3,-0.72},{36.15,-0.72},{36.15,0},{44,0}}, color={95,95,95}));
    connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{59,0},{52,0}}, color={95,95,95}));
    connect(cold_liquid_sink.C_in, Q_cold_liquid_out.C_out) annotation (Line(points={{-83,-52},{-50,-52},{-50,-44}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, Q_cold_liquid_out.C_in) annotation (Line(points={{-46,44},{-50,44},{-50,-28}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, P_cold_out_sensor.C_in) annotation (Line(points={{-46,52},{-58,52}}, color={28,108,200}));
    connect(cold_steam_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{-85,52},{-74,52}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,76},{-4,40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-36,56},{-34,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,58},{-28,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,52},{-30,50}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
            Rectangle(
            extent={{-60,40},{78,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_noRecirculation_reverse;

  model Evaporator_withRecirculation_direct
                // Boundary conditions
    input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
    input Units.MassFlowRate Q_hot_source(start=586);
    input Real hot_source_h(start=494000);

    input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
    input Units.MassFlowRate Q_cold_source(start=96);
    input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

     // Parameters
    parameter Units.Area S = 10;
    parameter Units.HeatExchangeCoefficient Kth= 102000;
    parameter Units.FrictionCoefficient Kfr_hot = 0;
    parameter Units.FrictionCoefficient Kfr_cold = 1;

    MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-18,46},
              {16,80}})));
    WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{54,38},
              {74,58}})));
    WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{54,70},
              {74,90}})));
    FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-10,-10},
              {10,10}},
          rotation=270,
          origin={-28,88})));
    FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
    WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-46,44})));
  equation

    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source;
    hot_source.h_out = hot_source_h;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source;
    cold_source.T_out =  T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    evaporator.S_vaporising = S;
    evaporator.Kth = Kth;
    evaporator.Kfr_hot = Kfr_hot;
    evaporator.Kfr_cold = Kfr_cold;

    // Recirculation flow
    evaporator.Q_cold = Q_cold_source;


    connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
    connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{28.3,-0.72},{45.65,-0.72},{45.65,0},{59,0}}, color={95,95,95}));
    connect(cold_source.C_out, flashTank.C_in) annotation (Line(points={{-28,83},{
            -28,69.8},{-18,69.8}},                   color={28,108,200}));
    connect(flashTank.C_hot_steam, cold_steam_sink.C_in) annotation (Line(points={
            {16,69.8},{16,68},{48,68},{48,80},{59,80}}, color={28,108,200}));
    connect(cold_liquid_sink.C_in, flashTank.C_hot_liquid)
      annotation (Line(points={{59,48},{16,48},{16,56.2}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, evaporator.C_cold_in) annotation (Line(points=
           {{16,56.2},{16,25.2},{12.7,25.2}}, color={28,108,200}));
    connect(evaporator.C_cold_out, pressureCut.C_in) annotation (Line(points={{-10.7,
            25.2},{-46,25.2},{-46,34}}, color={28,108,200}));
    connect(pressureCut.C_out, flashTank.C_in) annotation (Line(points={{-46,54},{
            -48,54},{-48,69.8},{-18,69.8}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,76},{-4,40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-36,56},{-34,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,58},{-28,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,52},{-30,50}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
            Rectangle(
            extent={{-60,40},{78,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_withRecirculation_direct;

end CCGT;
