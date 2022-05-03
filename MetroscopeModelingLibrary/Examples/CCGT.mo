within MetroscopeModelingLibrary.Examples;
package CCGT
  package MetroscopiaCCGT

    model MetroscopiaCCGT_reverse
      extends Icons.Tests.MultifluidTestIcon;

      // Boundary conditions

        // Air source
        input Units.Pressure P_source_air(start=1) "bar";
        input Units.NegativeMassFlowRate Q_source_air(start=500) "kg/s";
        input Real T_source_air(start=24) "degC";
        // Fuel source
        input Units.Pressure P_fuel_source(start = 30) "bar";
        input Real T_fuel_source(start=156);
        // Circulating water circuit
        input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
        input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
        // Flue gas sink
        input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";

      // Parameters

        // Gas Turbine
        parameter Units.SpecificEnthalpy LHV = 48130e3;
        parameter Real GT_h_out = 1e6; // This corresponds to T = 640°C at 1.1 bar
        // Economizer
        parameter String Eco_QCp_max_side = "hot";
        parameter Real T_w_eco_in = 85; // This temperature is controlled by the economizer recirculation pump flow rate
        // High Pressure Superheater
        parameter String HPSH_QCp_max_side = "hot";
        // Low Pressure Superheater (resuperheater)
        parameter String LPSH_QCp_max_side = "hot";

      // Observables used for calibration

        // Gas Turbine
        input Real compressor_P_out(start = 17) "barA";
        input Real compressor_T_out(start = 450) "degC";
        input Real W_GT(start = 150) "MW";
        input Real turbine_P_out(start=1.1) "barA";
        // Economizer
        input Real P_w_eco_out(start = 122.5, min= 0, nominal = 122.5) "barA";
        input Real T_w_eco_out(start = 320, min = 0, nominal = 320) "degC";
        // Evaporator
        input Real P_w_evap_out(start = 120, min= 0, nominal = 120) "barA";
        input Real Evap_x_steam_out(start = 1);
        // High Pressure Superheater
        input Real P_w_HPSH_out(start=114, min=0, nominal=114) "barA";
        input Real T_w_HPSH_out(start=566.5, min= 200, nominal=566.5) "degC";
        // Low Pressure Superheater (resuperheater)
        input Real T_w_LPSH_out(start = 350, min = 0, nominal = 350) "degC";
        input Real P_w_LPSH_out(start=9, min=0, nominal=9) "barA";
        // High Pressure Steam Turbine
        input Real HPST_opening(start=0.35);
        input Real P_ST_in(start=113);
        input Real P_ST_out(start=10, unit="bar", nominal=10, min=0) "bar";
        input Real W_ST_out(start=65, unit="MW", nominal=65, min=0) "MW";
        input Real LPST_opening(start=0.35);
        input Real P_LPST_in(start=8, min=0, nominal=4.9) "bar";
        // Condenser
        input Real P_Cond(start=0.05, min=0, nominal=0.05) "bar";
        input Real T_circulating_water_out(start=25, min=10, nominal=25) "degC";
        // Feed Pump
        input Real P_pump_out(start=170) "barA";
        input Real T_pump_out(start=35) "degC";
        input Real Q_pump_out(start=50);
        // Recirculation Pump
        input Real P_pumpRec_out(start=180) "barA";
        input Real T_pumpRec_out(start=324) "degC";
        input Real pumpRec_opening(start=0.35);

      // Calibrated parameters (input used for calibration in comment)

        // Gas Turbine
        output Real compression_rate; // Air compressor outlet pressure
        output Real compressor_eta_is; // Air compressor outlet temperature
        output Real turbine_compression_rate; // Gas turbine outlet pressure
        output Real turbine_eta_is; // Gas turbine power output
        output Units.NegativeMassFlowRate Q_fuel_source; // Observable: controlled by the gas turbine outlet temperature
        // Economizer
        output Units.HeatExchangeCoefficient Eco_Kth; // Economizer water outlet temperature
        output Units.FrictionCoefficient Eco_Kfr_hot; // Economizer flue gas outlet pressure
        output Units.FrictionCoefficient Eco_Kfr_cold; // Economizer water outlet pressure
        output Real T_flue_gas_sink; // Observable
        // Evaporator
        output Units.HeatExchangeCoefficient Evap_Kth; // Feed pump mass flow rate
        output Units.FrictionCoefficient Evap_Kfr_cold; // Evaporator outlet pressure
        // High Pressure Superheater
        output Units.HeatExchangeCoefficient HPSH_Kth; // HP superheater temperature
        output Units.FrictionCoefficient HPSH_Kfr_cold; // HP superheater outlet pressure
        // Low Pressure Superheater (resuperheater)
        output Units.HeatExchangeCoefficient LPSH_Kth; // LP superheater outlet temperature
        output Units.FrictionCoefficient LPSH_Kfr_cold; // LP superheater outlet pressure
        // High Pressure Steam Turbine
        output Units.Cv HPST_CV_Cvmax "Cvmax"; // HP steam turbine admission valve opening
        output Units.Cst HPST_Cst; // HP steam turbine inlet pressure
        output Units.Yield HPST_eta_is; // Power output and HPST_eta_is = LPST_eta_is
        // Low Pressure Steam Turbine
        output Units.Cv LPST_CV_Cvmax "Cvmax"; // LP steam turbine admission valve opening
        output Units.Cst LPST_Cst; // LP steam turbine inlet pressure
        output Units.Yield LPST_eta_is; // Power output and HPST_eta_is = LPST_eta_is
        // Condenser
        output Units.HeatExchangeCoefficient Cond_Kth; // Condensation pressure
        output Units.VolumeFlowRate Qv_cond_cold; // Circulating water outlet temperature
        // Feed Pump
        output Real pump_a3; // Feed pump outlet pressure
        output Real pump_b3; // Feed pump outlet temperature
        // Recirculation pump
        output Real pumpRec_a3; // Recirculation pump outlet pressure
        output Real pumpRec_b3; // Recirculation pump outlet temperature
        output Real Q_pumpRec_out; // Observable: controlled by the economizer input temperature
        output Real pumpRec_CV_Cvmax; // Recirculation control valve opening


      MultiFluid.HeatExchangers.Economiser economiser(
          QCp_max_side=Eco_QCp_max_side)
        annotation (Placement(transformation(extent={{-8,-55},{50,3}})));
      FlueGases.BoundaryConditions.Sink flue_gas_sink annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={140,198})));
      Sensors.WaterSteam.TemperatureSensor T_w_eco_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-50,8})));
      Sensors.WaterSteam.PressureSensor P_w_eco_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-28,8})));
      MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-104,
                -56},{-46,4.5}})));
      Sensors.WaterSteam.PressureSensor P_w_evap_out_sensor
        annotation (Placement(transformation(extent={{-194,2},{-206,14}})));
      MultiFluid.HeatExchangers.Superheater HPsuperheater(
          QCp_max_side=HPSH_QCp_max_side)
        annotation (Placement(transformation(extent={{-270,-56},{-210,4}})));
      Sensors.WaterSteam.TemperatureSensor T_w_HPSH_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-264,8})));
      Sensors.WaterSteam.PressureSensor P_w_HPSH_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-286,8})));
      WaterSteam.Pipes.ControlValve HPST_control_valve
        annotation (Placement(transformation(extent={{-285.25,108.738},{-268.75,126.677}})));
      Sensors.Outline.OpeningSensor HPST_opening_sensor
        annotation (Placement(transformation(extent={{-282,138},{-272,148}})));
      Sensors.WaterSteam.PressureSensor P_HPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-256,112})));
      WaterSteam.Machines.StodolaTurbine HPsteamTurbine
        annotation (Placement(transformation(extent={{-242,96},{-208,128}})));
      Sensors.WaterSteam.PressureSensor P_HPST_out_sensor
        annotation (Placement(transformation(extent={{-196,106},{-184,118}})));
      Sensors.Power.PowerSensor W_ST_out_sensor
        annotation (Placement(transformation(extent={{8,214},{20,226}})));
      WaterSteam.HeatExchangers.Condenser  condenser annotation (Placement(transformation(extent={{-50,
                108.778},{-10,140.778}})));
      Sensors.WaterSteam.TemperatureSensor T_circulating_water_out_sensor
        annotation (Placement(transformation(extent={{4,135},{14,145}})));
      WaterSteam.BoundaryConditions.Source circulating_water_source annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-98,123})));
      WaterSteam.BoundaryConditions.Sink circulating_water_sink
        annotation (Placement(transformation(extent={{16,132},{32,148}})));
      WaterSteam.Machines.Pump  pump annotation (Placement(transformation(extent={{-7,-7},
                {7,7}},                                                                                                         origin={34,95},
            rotation=0)));
      Sensors.WaterSteam.TemperatureSensor T_pump_out_sensor annotation (Placement(transformation(extent={{5,-5},{
                -5,5}},
            rotation=180,
            origin={55,95})));
      Sensors.WaterSteam.PressureSensor P_pump_out_sensor annotation (Placement(transformation(extent={{-5,-5},
                {5,5}},                                                                                                                              origin={73,95})));
      Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={34,114})));
      WaterSteam.Pipes.LoopBreaker loopBreaker
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={100,28})));
      Sensors.WaterSteam.FlowSensor Q_pump_out_sensor
        annotation (Placement(transformation(extent={{84,90},{94,100}})));
      FlueGases.BoundaryConditions.Source source_air( h_out(start=0.3e6)) annotation (Placement(transformation(extent={{-606,
                -36},{-586,-16}})));
      FlueGases.Machines.AirCompressor airCompressor(h_out(
            start=7e5))                                                                          annotation (Placement(transformation(extent={{-524,
                -40},{-496,-12}})));
      FlueGases.Machines.GasTurbine    gasTurbine(eta_is(
            start=0.73), eta_mech(start=0.9), h_out(
            start=0.5e6))                                   annotation (Placement(transformation(extent={{-414,
                -42},{-382,-10}})));
      Power.BoundaryConditions.Sink sink_power annotation (Placement(transformation(extent={{-332,24},
                {-312,44}})));
      MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-452,
                -36},{-432,-16}})));
      Fuel.BoundaryConditions.Source source_fuel(h_out(start=0.9e6)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-442,-90})));
      Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{-490,
                -32},{-478,-20}})));
      Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{-472,
                -32},{-460,-20}})));
      Sensors.FlueGases.PressureSensor turbine_P_out_sensor annotation (Placement(transformation(extent={{-350,
                -32},{-338,-20}})));
      Sensors.Power.PowerSensor W_GT_sensor
        annotation (Placement(transformation(extent={{-346,28},{-334,40}})));
      Sensors.FlueGases.TemperatureSensor turbine_T_out_sensor
        annotation (Placement(transformation(extent={{-370,-32},{-358,-20}})));
      MultiFluid.HeatExchangers.Superheater LPsuperheater(
          QCp_max_side=LPSH_QCp_max_side)
        annotation (Placement(transformation(extent={{-184,-56},{-124,4}})));
      WaterSteam.Machines.StodolaTurbine LPsteamTurbine
        annotation (Placement(transformation(extent={{-96,162},{-62,194}})));
      Sensors.WaterSteam.TemperatureSensor T_w_LPSH_out_sensor
        annotation (Placement(transformation(
            extent={{6,-6},{-6,6}},
            rotation=270,
            origin={-162,55})));
      Sensors.WaterSteam.PressureSensor P_w_LPSH_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={-162,75})));
      Sensors.WaterSteam.PressureSensor P_Cond_sensor
        annotation (Placement(transformation(extent={{-54,172},{-42,184}})));
      Sensors.FlueGases.PressureSensor P_source_air_sensor
        annotation (Placement(transformation(extent={{-584,-32},{-572,-20}})));
      Sensors.FlueGases.TemperatureSensor T_source_air_sensor
        annotation (Placement(transformation(extent={{-566,-32},{-554,-20}})));
      Sensors.FlueGases.FlowSensor Q_source_air_sensor
        annotation (Placement(transformation(extent={{-548,-32},{-536,-20}})));
      Sensors.WaterSteam.TemperatureSensor T_circulating_water_in_sensor
        annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=180,
            origin={-79,123})));
      Sensors.WaterSteam.PressureSensor P_circulating_water_in_sensor annotation (
          Placement(transformation(extent={{-5,-5},{5,5}}, origin={-63,123})));
      WaterSteam.Pipes.ControlValve LPST_control_valve
        annotation (Placement(transformation(extent={{-143.25,174.738},{-126.75,192.677}})));
      Sensors.WaterSteam.PressureSensor P_LPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-110,178})));
      Sensors.Outline.OpeningSensor LPST_opening_sensor
        annotation (Placement(transformation(extent={{-140,200},{-130,210}})));
      WaterSteam.Machines.Pump pumpRec(Q_in_0=1)                 annotation (
          Placement(transformation(
            extent={{-7,-7},{7,7}},
            origin={12,48.5455},
            rotation=0)));
      Power.BoundaryConditions.Source pumpRec_powerSource
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={12,66})));
      Sensors.WaterSteam.TemperatureSensor T_pumpRec_out_sensor
        annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=180,
            origin={33,48.5455})));
      Sensors.WaterSteam.PressureSensor P_pumpRec_out_sensor
        annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={49,48.5455})));
      Sensors.WaterSteam.FlowSensor Q_pumpRec_out_sensor
        annotation (Placement(transformation(extent={{58,43.5455},{68,53.5455}})));
      Sensors.WaterSteam.TemperatureSensor T_w_eco_in_sensor
        annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=180,
            origin={63,9})));
      WaterSteam.Pipes.ControlValve pumpRec_controlValve
        annotation (Placement(transformation(extent={{75,46},{88,60}})));
      Sensors.Outline.OpeningSensor pumpRec_opening_sensor
        annotation (Placement(transformation(extent={{76,68},{86,78}})));
      Sensors.FlueGases.PressureSensor P_flue_gas_sink_sensor annotation (Placement(
            transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={140,172})));
      Power.BoundaryConditions.Sink sink
        annotation (Placement(transformation(extent={{28,210},{48,230}})));
      Sensors.Fuel.PressureSensor P_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-61})));
      Sensors.Fuel.TemperatureSensor T_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-75})));
      Sensors.Fuel.FlowSensor Q_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-47})));
      Power.Machines.Generator GT_generator
        annotation (Placement(transformation(extent={{-380,24},{-348,44}})));
      Power.Machines.Generator ST_generator
        annotation (Placement(transformation(extent={{-32,210},{0,230}})));
      Sensors.FlueGases.TemperatureSensor T_flue_gas_sink_sensor annotation (
          Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={140,156})));
    equation
      // Quantities definition
      P_circulating_water_in_sensor.P_barA = P_circulating_water_in;
      T_circulating_water_in_sensor.T_degC = T_circulating_water_in;


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
          // Parameters
          evaporator.S_vaporising = 100;
          evaporator.Kfr_hot = 0;
          // Calibrated parameters
          evaporator.Kth = Evap_Kth;
          evaporator.Kfr_cold = Evap_Kfr_cold;

        // HP Superheater
          // Quantities definition
          P_w_HPSH_out_sensor.P_barA = P_w_HPSH_out;
          // Parameters
          HPsuperheater.S = 100;
          HPsuperheater.nominal_cold_side_temperature_rise = 250;
          HPsuperheater.nominal_hot_side_temperature_rise = 180;
          HPsuperheater.Kfr_hot = 0;
          T_w_HPSH_out_sensor.T_degC = T_w_HPSH_out;
          // Calibrated parameters
          HPsuperheater.Kth = HPSH_Kth;
          HPsuperheater.Kfr_cold = HPSH_Kfr_cold;

        // LP Superheater
          // Quantities definition
          T_w_LPSH_out_sensor.T_degC = T_w_LPSH_out;
          P_w_LPSH_out_sensor.P_barA = P_w_LPSH_out;
          // Parameters
          LPsuperheater.S = 100;
          LPsuperheater.nominal_cold_side_temperature_rise = 100;
          LPsuperheater.nominal_hot_side_temperature_rise = 180;
          LPsuperheater.Kfr_hot = 0;
          // Calibrated parameters
          LPsuperheater.Kth = LPSH_Kth;
          LPsuperheater.Kfr_cold = LPSH_Kfr_cold;

        // Steam Turbines

          // Steam Turbine Generator
            // Quantities definition
            W_ST_out_sensor.W_MW = W_ST_out;
            // Parameters
            ST_generator.eta = 0.99;

          // High Pressure Level
            // Quantities definition
            HPST_opening_sensor.Opening = HPST_opening;
            P_HPST_in_sensor.P_barA = P_ST_in;
            P_HPST_out_sensor.P_barA = P_ST_out;
            // Parameters
            HPsteamTurbine.area_nz = 1;
            HPsteamTurbine.eta_nz = 1;
            HPsteamTurbine.eta_is = LPsteamTurbine.eta_is;
            // Calibrated Parameters
            HPST_control_valve.Cvmax = HPST_CV_Cvmax;
            HPsteamTurbine.Cst = HPST_Cst;
            HPsteamTurbine.eta_is = HPST_eta_is;

          // Low Pressure Level
            // Quantities definition
            LPST_opening_sensor.Opening = LPST_opening;
            P_LPST_in_sensor.P_barA = P_LPST_in;
            // Parameters
            LPsteamTurbine.area_nz = 1;
            LPsteamTurbine.eta_nz = 1;
            // Calibrated Parameters
            LPST_control_valve.Cvmax = LPST_CV_Cvmax;
            LPsteamTurbine.Cst = LPST_Cst;
            LPsteamTurbine.eta_is = LPST_eta_is;

        // Condenser
          // Quantities definition
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

        // Feed Pump
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

      connect(T_w_eco_out_sensor.C_in, P_w_eco_out_sensor.C_out)
        annotation (Line(points={{-44,8},{-34,8}},   color={28,108,200}));
      connect(HPsuperheater.C_cold_out, T_w_HPSH_out_sensor.C_in) annotation (Line(
            points={{-249,-5},{-250,-5},{-250,8},{-258,8}},   color={28,108,200}));
      connect(HPST_control_valve.C_in, P_w_HPSH_out_sensor.C_out) annotation (Line(
            points={{-285.25,112},{-304,112},{-304,8},{-292,8}},   color={28,108,200}));
      connect(P_HPST_out_sensor.C_in, HPsteamTurbine.C_out)
        annotation (Line(points={{-196,112},{-208,112}}, color={28,108,200}));
      connect(P_HPST_in_sensor.C_out, HPsteamTurbine.C_in)
        annotation (Line(points={{-250,112},{-242,112}}, color={28,108,200}));
      connect(T_circulating_water_out_sensor.C_out, circulating_water_sink.C_in)
        annotation (Line(points={{14,140},{20,140}}, color={28,108,200}));
      connect(combustionChamber.outlet,gasTurbine. C_in) annotation (Line(points={{-432,
              -26},{-414,-26}},                                                                   color={95,95,95}));
      connect(airCompressor.C_out,compressor_P_out_sensor. C_in) annotation (Line(points={{-496,
              -26},{-490,-26}},                                                                             color={95,95,95}));
      connect(compressor_P_out_sensor.C_out,compressor_T_out_sensor. C_in) annotation (Line(points={{-478,
              -26},{-472,-26}},                                                                                       color={95,95,95}));
      connect(compressor_T_out_sensor.C_out,combustionChamber. inlet) annotation (Line(points={{-460,
              -26},{-452,-26}},                                                                                  color={95,95,95}));
      connect(turbine_P_out_sensor.C_out, HPsuperheater.C_hot_in)
        annotation (Line(points={{-338,-26},{-261,-26}}, color={95,95,95}));
      connect(evaporator.C_cold_in, T_w_eco_out_sensor.C_out) annotation (Line(
            points={{-66.3,-4.575},{-66.3,8},{-56,8}},             color={28,108,200}));
      connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in)
        annotation (Line(points={{-10,121.934},{-8,121.934},{-8,122},{-4,122},{
              -4,140},{4,140}},
                        color={28,108,200}));
      connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{-30,
              108.067},{-30,95},{27,95}},
                                  color={28,108,200}));
      connect(powerSource.C_out, pump.C_power)
        annotation (Line(points={{34,109.2},{34,102.56}},
                                                       color={244,125,35}));
      connect(HPsuperheater.C_cold_in, P_w_evap_out_sensor.C_out) annotation (Line(
            points={{-231,-5},{-231,8},{-206,8}},   color={28,108,200}));
      connect(evaporator.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
            points={{-83.7,-4.575},{-82,-4.575},{-82,8},{-194,8}},
                                                            color={28,108,200}));
      connect(economiser.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
            points={{12.3,-5.7},{12,-5.7},{12,8},{-22,8}},
                                                       color={28,108,200}));
      connect(evaporator.C_hot_out, economiser.C_hot_in) annotation (Line(points={{-54.7,
              -26.355},{-31,-26.355},{-31,-26},{0.7,-26}},   color={95,95,95}));
      connect(gasTurbine.C_W_compressor, airCompressor.C_W_in) annotation (Line(
          points={{-414,-10},{-414,16},{-496,16},{-496,-12}},
          color={244,125,35},
          smooth=Smooth.Bezier));
      connect(gasTurbine.C_out, turbine_T_out_sensor.C_in)
        annotation (Line(points={{-382,-26},{-370,-26}}, color={95,95,95}));
      connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out)
        annotation (Line(points={{-350,-26},{-358,-26}}, color={95,95,95}));
      connect(evaporator.C_hot_in, LPsuperheater.C_hot_out) annotation (Line(points={{-95.3,
              -26.355},{-95.3,-26},{-133,-26}},        color={95,95,95}));
      connect(HPsuperheater.C_hot_out, LPsuperheater.C_hot_in)
        annotation (Line(points={{-219,-26},{-175,-26}}, color={95,95,95}));
      connect(LPsuperheater.C_cold_out, T_w_LPSH_out_sensor.C_in) annotation (Line(
            points={{-163,-5},{-162,-5},{-162,49}},           color={28,108,200}));
      connect(P_Cond_sensor.C_in, LPsteamTurbine.C_out)
        annotation (Line(points={{-54,178},{-62,178}}, color={28,108,200}));
      connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{-42,178},
              {-30,178},{-30,140.778}},  color={28,108,200}));

      connect(P_HPST_out_sensor.C_out, LPsuperheater.C_cold_in) annotation (Line(
            points={{-184,112},{-145,112},{-145,-5}}, color={28,108,200}));
      connect(source_air.C_out, P_source_air_sensor.C_in)
        annotation (Line(points={{-591,-26},{-584,-26}}, color={95,95,95}));
      connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
        annotation (Line(points={{-572,-26},{-566,-26}}, color={95,95,95}));
      connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
        annotation (Line(points={{-554,-26},{-548,-26}}, color={95,95,95}));
      connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
         Line(points={{-50.8,131.889},{-52,131.889},{-52,123},{-58,123}}, color={28,
              108,200}));
      connect(P_LPST_in_sensor.C_out, LPsteamTurbine.C_in)
        annotation (Line(points={{-104,178},{-96,178}}, color={28,108,200}));
      connect(P_LPST_in_sensor.C_in, LPST_control_valve.C_out) annotation (Line(
            points={{-116,178},{-122,178},{-122,178},{-126.75,178}},         color={
              28,108,200}));
      connect(P_w_LPSH_out_sensor.C_out, LPST_control_valve.C_in) annotation (Line(
            points={{-162,81},{-162,178},{-143.25,178}}, color={28,108,200}));
      connect(P_w_LPSH_out_sensor.C_in, T_w_LPSH_out_sensor.C_out)
        annotation (Line(points={{-162,69},{-162,61}}, color={28,108,200}));
      connect(T_w_HPSH_out_sensor.C_out, P_w_HPSH_out_sensor.C_in)
        annotation (Line(points={{-270,8},{-280,8}},   color={28,108,200}));
      connect(LPST_control_valve.Opening, LPST_opening_sensor.Opening)
        annotation (Line(points={{-135,191.046},{-135,199.9}}, color={0,0,127}));
      connect(HPST_control_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
            points={{-268.75,112},{-266,112},{-266,112},{-262,112}}, color={28,108,200}));
      connect(HPST_control_valve.Opening, HPST_opening_sensor.Opening)
        annotation (Line(points={{-277,125.046},{-277,137.9}}, color={0,0,127}));
      connect(T_circulating_water_in_sensor.C_out, P_circulating_water_in_sensor.C_in)
        annotation (Line(points={{-74,123},{-68,123}}, color={28,108,200}));
      connect(circulating_water_source.C_out, T_circulating_water_in_sensor.C_in)
        annotation (Line(points={{-93,123},{-84,123},{-84,123}}, color={28,108,200}));
      connect(pump.C_out, T_pump_out_sensor.C_in)
        annotation (Line(points={{41,95},{50,95},{50,95}}, color={28,108,200}));
      connect(T_pump_out_sensor.C_out, P_pump_out_sensor.C_in) annotation (Line(
            points={{60,95},{62,95},{62,95},{68,95}}, color={28,108,200}));
      connect(P_pump_out_sensor.C_out, Q_pump_out_sensor.C_in) annotation (Line(
            points={{78,95},{80,95},{80,95},{84,95}}, color={28,108,200}));
      connect(pumpRec.C_power, pumpRec_powerSource.C_out)
        annotation (Line(points={{12,56.1055},{12,61.2}},
                                                        color={244,125,35}));
      connect(P_pumpRec_out_sensor.C_out, Q_pumpRec_out_sensor.C_in)
        annotation (Line(points={{54,48.5455},{58,48.5455}},
                                                   color={28,108,200}));
      connect(pumpRec_controlValve.Opening, pumpRec_opening_sensor.Opening)
        annotation (Line(points={{81.5,58.7273},{81.5,62},{81,62},{81,67.9}},
                                                                    color={0,0,127}));
      connect(pumpRec.C_in, P_w_eco_out_sensor.C_in) annotation (Line(points={{5,48.5455},
              {-8,48.5455},{-8,8},{-22,8}},
                                       color={28,108,200}));
      connect(economiser.C_cold_in, T_w_eco_in_sensor.C_out) annotation (Line(
            points={{29.7,-5.7},{29.7,9},{58,9}}, color={28,108,200}));
      connect(T_w_eco_in_sensor.C_in, loopBreaker.C_out) annotation (Line(points={{68,9},{
              68,8},{100,8},{100,18}},     color={28,108,200}));
      connect(Q_pump_out_sensor.C_out, loopBreaker.C_in)
        annotation (Line(points={{94,95},{100,95},{100,38}}, color={28,108,200}));
      connect(T_pumpRec_out_sensor.C_out, P_pumpRec_out_sensor.C_in) annotation (
          Line(points={{38,48.5455},{44,48.5455}},                           color={
              28,108,200}));
      connect(Q_pumpRec_out_sensor.C_out, pumpRec_controlValve.C_in)
        annotation (Line(points={{68,48.5455},{72,48.5455},{72,48.5455},{75,
              48.5455}},                                     color={28,108,200}));
      connect(flue_gas_sink.C_in, P_flue_gas_sink_sensor.C_out)
        annotation (Line(points={{140,193},{140,178}}, color={95,95,95}));
      connect(sink.C_in, W_ST_out_sensor.C_out)
        annotation (Line(points={{33,220},{19.88,220}},  color={244,125,35}));
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
        annotation (Line(points={{8,220},{-4.8,220}}, color={244,125,35}));
      connect(HPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
          points={{-208,125.44},{-208,125.44},{-208,158},{-208,220},{-25.92,220}},
          color={244,125,35},
          smooth=Smooth.Bezier));
      connect(LPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
          points={{-62,191.44},{-62,220},{-25.92,220}},
          color={244,125,35},
          smooth=Smooth.Bezier));
      connect(P_flue_gas_sink_sensor.C_in, T_flue_gas_sink_sensor.C_out)
        annotation (Line(points={{140,166},{140,162}}, color={95,95,95}));
      connect(T_flue_gas_sink_sensor.C_in, economiser.C_hot_out) annotation (Line(
            points={{140,150},{140,-26},{41.3,-26}},                     color={95,95,
              95}));
      connect(pumpRec_controlValve.C_out, loopBreaker.C_in) annotation (Line(points={{88,
              48.5455},{92,48.5455},{92,50},{100,50},{100,38}},     color={28,108,200}));
      connect(pumpRec.C_out, T_pumpRec_out_sensor.C_in) annotation (Line(points={{19,
              48.5455},{28,48.5455},{28,48.5455}}, color={28,108,200}));
      connect(Q_source_air_sensor.C_out, airCompressor.C_in)
        annotation (Line(points={{-536,-26},{-524,-26}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-820,-100},
                {200,240}})),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-820,-100},{200,240}}),
            graphics={Rectangle(
              extent={{-324,18},{164,-72}},
              pattern=LinePattern.None,
              lineColor={0,0,0},
              fillColor={158,158,158},
              fillPattern=FillPattern.Solid), Text(
              extent={{-174,-56},{-28,-68}},
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
              extent={{164,16},{118,190}},
              pattern=LinePattern.None,
              fillColor={158,158,158},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-586,-18},{-534,-34}},
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
              origin={-71,123},
              rotation=360),
            Rectangle(
              extent={{-8,8},{8,-8}},
              fillColor={255,82,82},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              origin={140,172},
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
              extent={{56,16},{70,2}},
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
              extent={{56,56},{70,42}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={244,237,30},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{132,164},{148,148}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor={244,237,30},
              fillPattern=FillPattern.Solid)}));
    end MetroscopiaCCGT_reverse;

    model MetroscopiaCCGT_deSH_reverse
      extends Icons.Tests.MultifluidTestIcon;

      // Boundary conditions

        // Air source
        input Units.Pressure P_source_air(start=1) "bar";
        input Units.NegativeMassFlowRate Q_source_air(start=500) "kg/s";
        input Real T_source_air(start=24) "degC";
        // Fuel source
        input Units.Pressure P_fuel_source(start = 30) "bar";
        input Real T_fuel_source(start=156);
        // Circulating water circuit
        input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
        input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
        // Flue gas sink
        input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";

      // Parameters

        // Gas Turbine
        parameter Units.SpecificEnthalpy LHV = 48130e3;
        parameter Real GT_h_out = 1e6; // This enthalpy corresponds to T = 640°C at 1.1 bar
        // Economizer
        parameter String Eco_QCp_max_side = "hot";
        parameter Real T_w_eco_in = 85; // This temperature is controlled by the economizer recirculation pump flow rate
        // High Pressure Superheater
        parameter String HPSH_QCp_max_side = "hot";
        // High Pressure Superheater 2
        parameter Real T_w_HPSH2_out = 566.5 "degC"; // This temperature is controlled by the desuperheater mass flow rate
        // Low Pressure Superheater (resuperheater)
        parameter String LPSH_QCp_max_side = "hot";

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
        input Real P_w_evap_out(start = 120, min= 0, nominal = 120) "barA";
        input Real Evap_x_steam_out(start = 1);
        // High Pressure Superheater 1
        input Real P_w_HPSH1_out(start=116, min=0, nominal=116) "barA";
        input Real T_w_HPSH1_out(start=450, min= 200, nominal=450) "degC";
        // High Pressure Superheater 2
        input Real P_w_HPSH2_out(start=114, min=0, nominal=114) "barA";
        // De-superheater
        input Real deSH_opening(start=0.15);
        input Real Q_deSH(start=2);
        // Low Pressure Superheater (resuperheater)
        input Real T_w_LPSH_out(start = 350, min = 0, nominal = 350) "degC";
        input Real P_w_LPSH_out(start=9, min=0, nominal=9) "barA";
        // High Pressure Steam Turbine
        input Real HPST_opening(start=0.35);
        input Real P_ST_in(start=113);
        input Real P_ST_out(start=10, unit="bar", nominal=10, min=0) "bar";
        input Real W_ST_out(start=65, unit="MW", nominal=65, min=0) "MW";
        input Real LPST_opening(start=0.35);
        input Real P_LPST_in(start=8, min=0, nominal=4.9) "bar";
        // Condenser
        input Real P_Cond(start=0.05, min=0, nominal=0.05) "bar";
        input Real T_circulating_water_out(start=25, min=10, nominal=25) "degC";
        // Feed Pump
        input Real P_pump_out(start=170) "barA";
        input Real T_pump_out(start=35) "degC";
        input Real Q_pump_out(start=50);
        // Recirculation Pump
        input Real P_pumpRec_out(start=180) "barA";
        input Real T_pumpRec_out(start=324) "degC";
        input Real pumpRec_opening(start=0.35);

      // Calibrated parameters (input used for calibration in comment)

        // Gas Turbine
        output Units.FrictionCoefficient Filter_Kfr; // Filter outlet pressure
        output Real compression_rate; // Air compressor outlet pressure
        output Real compressor_eta_is; // Air compressor outlet temperature
        output Real turbine_compression_rate; // Gas turbine outlet pressure
        output Real turbine_eta_is; // Gas turbine power output
        output Units.NegativeMassFlowRate Q_fuel_source; // Observable: controlled by the gas turbine outlet temperature
        // Economizer
        output Units.HeatExchangeCoefficient Eco_Kth; // Economizer water outlet temperature
        output Units.FrictionCoefficient Eco_Kfr_hot; // Economizer flue gas outlet pressure
        output Units.FrictionCoefficient Eco_Kfr_cold; // Economizer water outlet pressure
        output Real T_flue_gas_sink; // Observable
        // Evaporator
        output Units.HeatExchangeCoefficient Evap_Kth; // Feed pump mass flow rate
        output Units.FrictionCoefficient Evap_Kfr_cold; // Evaporator outlet pressure
        // High Pressure Superheater 1
        output Units.HeatExchangeCoefficient HPSH1_Kth; // HP superheater temperature
        output Units.FrictionCoefficient HPSH1_Kfr_cold; // HP superheater outlet pressure
        // High Pressure Superheater 2
        output Units.HeatExchangeCoefficient HPSH2_Kth; // De-superheater mass flow rate
        output Units.FrictionCoefficient HPSH2_Kfr_cold; // HP superheater outlet pressure
        // De-superheater
        output Units.Cv deSH_CV_Cvmax "Cvmax"; // Desuperheater control valve opening
        // Low Pressure Superheater (resuperheater)
        output Units.HeatExchangeCoefficient LPSH_Kth; // LP superheater outlet temperature
        output Units.FrictionCoefficient LPSH_Kfr_cold; // LP superheater outlet pressure
        // High Pressure Steam Turbine
        output Units.Cv HPST_CV_Cvmax "Cvmax"; // HP steam turbine admission valve opening
        output Units.Cst HPST_Cst; // HP steam turbine inlet pressure
        output Units.Yield HPST_eta_is; // Power output and HPST_eta_is = LPST_eta_is
        // Low Pressure Steam Turbine
        output Units.Cv LPST_CV_Cvmax "Cvmax"; // LP steam turbine admission valve opening
        output Units.Cst LPST_Cst; // LP steam turbine inlet pressure
        output Units.Yield LPST_eta_is; // Power output and HPST_eta_is = LPST_eta_is
        // Condenser
        output Units.HeatExchangeCoefficient Cond_Kth; // Condensation pressure
        output Units.VolumeFlowRate Qv_cond_cold; // Circulating water outlet temperature
        // Feed Pump
        output Real pump_a3; // Feed pump outlet pressure
        output Real pump_b3; // Feed pump outlet temperature
        // Recirculation pump
        output Real pumpRec_a3; // Recirculation pump outlet pressure
        output Real pumpRec_b3; // Recirculation pump outlet temperature
        output Real Q_pumpRec_out; // Observable: controlled by the economizer input temperature
        output Real pumpRec_CV_Cvmax; // Recirculation control valve opening

      MultiFluid.HeatExchangers.Economiser economiser(
          QCp_max_side=Eco_QCp_max_side)
        annotation (Placement(transformation(extent={{74,-56},{132,3}})));
      FlueGases.BoundaryConditions.Sink flue_gas_sink annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={222,198})));
      Sensors.WaterSteam.TemperatureSensor T_w_eco_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={32,8})));
      Sensors.WaterSteam.PressureSensor P_w_eco_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={54,8})));
      MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-22,-56},
                {36,4.5}})));
      Sensors.WaterSteam.PressureSensor P_w_evap_out_sensor
        annotation (Placement(transformation(extent={{-112,2},{-124,14}})));
      MultiFluid.HeatExchangers.Superheater HPsuperheater1(QCp_max_side=
            HPSH_QCp_max_side)
        annotation (Placement(transformation(extent={{-186,-56},{-126,4}})));
      Sensors.WaterSteam.TemperatureSensor T_w_HPSH1_out_sensor annotation (
          Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-184,8})));
      Sensors.WaterSteam.PressureSensor P_w_HPSH1_out_sensor annotation (
          Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=180,
            origin={-208,8})));
      WaterSteam.Pipes.ControlValve HPST_control_valve
        annotation (Placement(transformation(extent={{-203.25,144.738},{-186.75,162.677}})));
      Sensors.Outline.OpeningSensor HPST_opening_sensor
        annotation (Placement(transformation(extent={{-200,174},{-190,184}})));
      Sensors.WaterSteam.PressureSensor P_HPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-174,148})));
      WaterSteam.Machines.StodolaTurbine HPsteamTurbine
        annotation (Placement(transformation(extent={{-160,132},{-126,164}})));
      Sensors.WaterSteam.PressureSensor P_HPST_out_sensor
        annotation (Placement(transformation(extent={{-114,142},{-102,154}})));
      Sensors.Power.PowerSensor W_ST_out_sensor
        annotation (Placement(transformation(extent={{90,250},{102,262}})));
      WaterSteam.HeatExchangers.Condenser  condenser annotation (Placement(transformation(extent={{32,
                144.778},{72,176.778}})));
      Sensors.WaterSteam.TemperatureSensor T_circulating_water_out_sensor
        annotation (Placement(transformation(extent={{86,171},{96,181}})));
      WaterSteam.BoundaryConditions.Source circulating_water_source annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-16,159})));
      WaterSteam.BoundaryConditions.Sink circulating_water_sink
        annotation (Placement(transformation(extent={{98,168},{114,184}})));
      WaterSteam.Machines.Pump  pump annotation (Placement(transformation(extent={{-7,-7},
                {7,7}},                                                                                                         origin={116,131},
            rotation=0)));
      Sensors.WaterSteam.TemperatureSensor T_pump_out_sensor annotation (Placement(transformation(extent={{5,-5},{
                -5,5}},
            rotation=180,
            origin={137,131})));
      Sensors.WaterSteam.PressureSensor P_pump_out_sensor annotation (Placement(transformation(extent={{-5,-5},
                {5,5}},                                                                                                                              origin={155,131})));
      Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={116,150})));
      WaterSteam.Pipes.LoopBreaker loopBreaker
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={182,28})));
      Sensors.WaterSteam.FlowSensor Q_pump_out_sensor
        annotation (Placement(transformation(extent={{166,126},{176,136}})));
      FlueGases.BoundaryConditions.Source source_air( h_out(start=0.3e6)) annotation (Placement(transformation(extent={{-658,
                -36},{-638,-16}})));
      FlueGases.Machines.AirCompressor airCompressor(h_out(
            start=7e5))                                                                          annotation (Placement(transformation(extent={{-524,
                -40},{-496,-12}})));
      FlueGases.Machines.GasTurbine    gasTurbine(eta_is(
            start=0.73), eta_mech(start=0.9), h_out(
            start=0.5e6))                                   annotation (Placement(transformation(extent={{-414,
                -42},{-382,-10}})));
      Power.BoundaryConditions.Sink sink_power annotation (Placement(transformation(extent={{-332,24},
                {-312,44}})));
      MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-452,
                -36},{-432,-16}})));
      Fuel.BoundaryConditions.Source source_fuel(h_out(start=0.9e6)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-442,-90})));
      Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{-490,
                -32},{-478,-20}})));
      Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{-472,
                -32},{-460,-20}})));
      Sensors.FlueGases.PressureSensor turbine_P_out_sensor annotation (Placement(transformation(extent={{-350,
                -32},{-338,-20}})));
      Sensors.Power.PowerSensor W_GT_sensor
        annotation (Placement(transformation(extent={{-346,28},{-334,40}})));
      Sensors.FlueGases.TemperatureSensor turbine_T_out_sensor
        annotation (Placement(transformation(extent={{-370,-32},{-358,-20}})));
      MultiFluid.HeatExchangers.Superheater LPsuperheater(
          QCp_max_side=LPSH_QCp_max_side)
        annotation (Placement(transformation(extent={{-102,-56},{-42,4}})));
      WaterSteam.Machines.StodolaTurbine LPsteamTurbine
        annotation (Placement(transformation(extent={{-14,198},{20,230}})));
      Sensors.WaterSteam.TemperatureSensor T_w_LPSH_out_sensor
        annotation (Placement(transformation(
            extent={{6,-6},{-6,6}},
            rotation=270,
            origin={-80,29})));
      Sensors.WaterSteam.PressureSensor P_w_LPSH_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={-80,49})));
      Sensors.WaterSteam.PressureSensor P_Cond_sensor
        annotation (Placement(transformation(extent={{28,208},{40,220}})));
      Sensors.FlueGases.PressureSensor P_source_air_sensor
        annotation (Placement(transformation(extent={{-636,-32},{-624,-20}})));
      Sensors.FlueGases.TemperatureSensor T_source_air_sensor
        annotation (Placement(transformation(extent={{-618,-32},{-606,-20}})));
      Sensors.FlueGases.FlowSensor Q_source_air_sensor
        annotation (Placement(transformation(extent={{-600,-32},{-588,-20}})));
      Sensors.WaterSteam.TemperatureSensor T_circulating_water_in_sensor
        annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=180,
            origin={3,159})));
      Sensors.WaterSteam.PressureSensor P_circulating_water_in_sensor annotation (
          Placement(transformation(extent={{-5,-5},{5,5}}, origin={19,159})));
      WaterSteam.Pipes.ControlValve LPST_control_valve
        annotation (Placement(transformation(extent={{-61.25,210.738},{-44.75,228.677}})));
      Sensors.WaterSteam.PressureSensor P_LPST_in_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=0,
            origin={-28,214})));
      Sensors.Outline.OpeningSensor LPST_opening_sensor
        annotation (Placement(transformation(extent={{-58,236},{-48,246}})));
      WaterSteam.Machines.Pump pumpRec(Q_in_0=1)                 annotation (
          Placement(transformation(
            extent={{-7,-7},{7,7}},
            origin={94,48.5455},
            rotation=0)));
      Power.BoundaryConditions.Source pumpRec_powerSource
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={94,66})));
      Sensors.WaterSteam.TemperatureSensor T_pumpRec_out_sensor
        annotation (Placement(transformation(
            extent={{5,-5},{-5,5}},
            rotation=180,
            origin={115,48.5455})));
      Sensors.WaterSteam.PressureSensor P_pumpRec_out_sensor
        annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={131,48.5455})));
      Sensors.WaterSteam.FlowSensor Q_pumpRec_out_sensor
        annotation (Placement(transformation(extent={{140,43.5455},{150,53.5455}})));
      Sensors.WaterSteam.TemperatureSensor T_w_eco_in_sensor
        annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=180,
            origin={145,9})));
      WaterSteam.Pipes.ControlValve pumpRec_controlValve
        annotation (Placement(transformation(extent={{157,46},{170,60}})));
      Sensors.Outline.OpeningSensor pumpRec_opening_sensor
        annotation (Placement(transformation(extent={{158,68},{168,78}})));
      Sensors.FlueGases.PressureSensor P_flue_gas_sink_sensor annotation (Placement(
            transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={222,172})));
      Power.BoundaryConditions.Sink sink
        annotation (Placement(transformation(extent={{110,246},{130,266}})));
      Sensors.Fuel.PressureSensor P_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-61})));
      Sensors.Fuel.TemperatureSensor T_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-75})));
      Sensors.Fuel.FlowSensor Q_fuel_source_sensor annotation (Placement(
            transformation(
            extent={{-5,-5},{5,5}},
            rotation=90,
            origin={-442,-47})));
      Power.Machines.Generator GT_generator
        annotation (Placement(transformation(extent={{-380,24},{-348,44}})));
      Power.Machines.Generator ST_generator
        annotation (Placement(transformation(extent={{50,246},{82,266}})));
      Sensors.FlueGases.TemperatureSensor T_flue_gas_sink_sensor annotation (
          Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={222,156})));
      FlueGases.Pipes.Filter AirFilter
        annotation (Placement(transformation(extent={{-576,-36},{-556,-16}})));
      Sensors.FlueGases.PressureSensor P_filter_out_sensor
        annotation (Placement(transformation(extent={{-548,-32},{-536,-20}})));
      MultiFluid.HeatExchangers.Superheater HPsuperheater2(QCp_max_side=
            HPSH_QCp_max_side)
        annotation (Placement(transformation(extent={{-302,-56},{-242,4}})));
      Sensors.WaterSteam.TemperatureSensor T_w_HPSH2_out_sensor
        annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={-282,34})));
      Sensors.WaterSteam.PressureSensor P_w_HPSH2_out_sensor annotation (Placement(
            transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={-282,52})));
      WaterSteam.Pipes.ControlValve deSH_controlValve annotation (Placement(
            transformation(extent={{-158.75,89.4545},{-171.25,103.455}})));
      Sensors.Outline.OpeningSensor deSH_opening_sensor
        annotation (Placement(transformation(extent={{-170,114},{-160,124}})));
      Sensors.WaterSteam.FlowSensor Q_deSH_sensor
        annotation (Placement(transformation(extent={{-132,86},{-144,98}})));
    equation
      // Quantities definition
      P_circulating_water_in_sensor.P_barA = P_circulating_water_in;
      T_circulating_water_in_sensor.T_degC = T_circulating_water_in;

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
          // Parameters
          evaporator.S_vaporising = 100;
          evaporator.Kfr_hot = 0;
          // Calibrated parameters
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

        // LP Superheater
          // Quantities definition
          T_w_LPSH_out_sensor.T_degC = T_w_LPSH_out;
          P_w_LPSH_out_sensor.P_barA = P_w_LPSH_out;
          // Parameters
          LPsuperheater.S = 100;
          LPsuperheater.nominal_cold_side_temperature_rise = 100;
          LPsuperheater.nominal_hot_side_temperature_rise = 180;
          LPsuperheater.Kfr_hot = 0;
          // Calibrated parameters
          LPsuperheater.Kth = LPSH_Kth;
          LPsuperheater.Kfr_cold = LPSH_Kfr_cold;

        // Steam Turbines

          // Steam Turbine Generator
            // Quantities definition
            W_ST_out_sensor.W_MW = W_ST_out;
            // Parameters
            ST_generator.eta = 0.99;

          // High Pressure Level
            // Quantities definition
            HPST_opening_sensor.Opening = HPST_opening;
            P_HPST_in_sensor.P_barA = P_ST_in;
            P_HPST_out_sensor.P_barA = P_ST_out;
            // Parameters
            HPsteamTurbine.area_nz = 1;
            HPsteamTurbine.eta_nz = 1;
            HPsteamTurbine.eta_is = LPsteamTurbine.eta_is;
            // Calibrated Parameters
            HPST_control_valve.Cvmax = HPST_CV_Cvmax;
            HPsteamTurbine.Cst = HPST_Cst;
            HPsteamTurbine.eta_is = HPST_eta_is;

          // Low Pressure Level
            // Quantities definition
            LPST_opening_sensor.Opening = LPST_opening;
            P_LPST_in_sensor.P_barA = P_LPST_in;
            // Parameters
            LPsteamTurbine.area_nz = 1;
            LPsteamTurbine.eta_nz = 1;
            // Calibrated Parameters
            LPST_control_valve.Cvmax = LPST_CV_Cvmax;
            LPsteamTurbine.Cst = LPST_Cst;
            LPsteamTurbine.eta_is = LPST_eta_is;

        // Condenser
          // Quantities definition
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

        // Feed Pump
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

      connect(T_w_eco_out_sensor.C_in, P_w_eco_out_sensor.C_out)
        annotation (Line(points={{38,8},{48,8}},     color={28,108,200}));
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
            points={{15.7,-4.575},{15.7,8},{26,8}},                color={28,108,200}));
      connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in)
        annotation (Line(points={{72,157.934},{74,157.934},{74,158},{78,158},{
              78,176},{86,176}},
                        color={28,108,200}));
      connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{52,
              144.067},{52,131},{109,131}},
                                  color={28,108,200}));
      connect(powerSource.C_out, pump.C_power)
        annotation (Line(points={{116,145.2},{116,138.56}},
                                                       color={244,125,35}));
      connect(HPsuperheater1.C_cold_in, P_w_evap_out_sensor.C_out) annotation (Line(
            points={{-147,-5},{-147,8},{-124,8}}, color={28,108,200}));
      connect(evaporator.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
            points={{-1.7,-4.575},{0,-4.575},{0,8},{-112,8}},
                                                            color={28,108,200}));
      connect(economiser.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
            points={{94.3,-5.85},{94,-5.85},{94,8},{60,8}},
                                                       color={28,108,200}));
      connect(evaporator.C_hot_out, economiser.C_hot_in) annotation (Line(points={{27.3,
              -26.355},{51,-26.355},{51,-26.5},{82.7,-26.5}},color={95,95,95}));
      connect(gasTurbine.C_W_compressor, airCompressor.C_W_in) annotation (Line(
          points={{-414,-10},{-414,16},{-496,16},{-496,-12}},
          color={244,125,35},
          smooth=Smooth.Bezier));
      connect(gasTurbine.C_out, turbine_T_out_sensor.C_in)
        annotation (Line(points={{-382,-26},{-370,-26}}, color={95,95,95}));
      connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out)
        annotation (Line(points={{-350,-26},{-358,-26}}, color={95,95,95}));
      connect(evaporator.C_hot_in, LPsuperheater.C_hot_out) annotation (Line(points={{-13.3,
              -26.355},{-13.3,-26},{-51,-26}},         color={95,95,95}));
      connect(HPsuperheater1.C_hot_out, LPsuperheater.C_hot_in)
        annotation (Line(points={{-135,-26},{-93,-26}}, color={95,95,95}));
      connect(LPsuperheater.C_cold_out, T_w_LPSH_out_sensor.C_in) annotation (Line(
            points={{-81,-5},{-80,-5},{-80,23}},              color={28,108,200}));
      connect(P_Cond_sensor.C_in, LPsteamTurbine.C_out)
        annotation (Line(points={{28,214},{20,214}},   color={28,108,200}));
      connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,214},
              {52,214},{52,176.778}},    color={28,108,200}));

      connect(P_HPST_out_sensor.C_out, LPsuperheater.C_cold_in) annotation (Line(
            points={{-102,148},{-63,148},{-63,-5}},   color={28,108,200}));
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
      connect(P_w_LPSH_out_sensor.C_out, LPST_control_valve.C_in) annotation (Line(
            points={{-80,55},{-80,214},{-61.25,214}},    color={28,108,200}));
      connect(P_w_LPSH_out_sensor.C_in, T_w_LPSH_out_sensor.C_out)
        annotation (Line(points={{-80,43},{-80,35}},   color={28,108,200}));
      connect(T_w_HPSH1_out_sensor.C_out, P_w_HPSH1_out_sensor.C_in)
        annotation (Line(points={{-190,8},{-202,8}}, color={28,108,200}));
      connect(LPST_control_valve.Opening, LPST_opening_sensor.Opening)
        annotation (Line(points={{-53,227.046},{-53,235.9}},   color={0,0,127}));
      connect(HPST_control_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
            points={{-186.75,148},{-184,148},{-184,148},{-180,148}}, color={28,108,200}));
      connect(HPST_control_valve.Opening, HPST_opening_sensor.Opening)
        annotation (Line(points={{-195,161.046},{-195,173.9}}, color={0,0,127}));
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
              {74,48.5455},{74,8},{60,8}},
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
        annotation (Line(points={{150,48.5455},{154,48.5455},{154,48.5455},{157,
              48.5455}},                                     color={28,108,200}));
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
      connect(P_flue_gas_sink_sensor.C_in, T_flue_gas_sink_sensor.C_out)
        annotation (Line(points={{222,166},{222,162}}, color={95,95,95}));
      connect(T_flue_gas_sink_sensor.C_in, economiser.C_hot_out) annotation (Line(
            points={{222,150},{222,-26.5},{123.3,-26.5}},                color={95,95,
              95}));
      connect(pumpRec_controlValve.C_out, loopBreaker.C_in) annotation (Line(points={{170,
              48.5455},{174,48.5455},{174,50},{182,50},{182,38}},   color={28,108,200}));
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
      connect(Q_deSH_sensor.C_out, deSH_controlValve.C_in) annotation (Line(points={{-144,92},
              {-152,92},{-152,92},{-158.75,92}},           color={28,108,200}));
      connect(deSH_controlValve.C_out, HPsuperheater2.C_cold_in) annotation (Line(
            points={{-171.25,92},{-230,92},{-230,8},{-263,8},{-263,-5}}, color={28,108,
              200}));
      connect(T_w_HPSH2_out_sensor.C_in, HPsuperheater2.C_cold_out) annotation (
          Line(points={{-282,28},{-282,12},{-282,-5},{-281,-5}}, color={28,108,200}));
      connect(P_w_HPSH2_out_sensor.C_in, T_w_HPSH2_out_sensor.C_out)
        annotation (Line(points={{-282,46},{-282,40}}, color={28,108,200}));
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
              extent={{214,164},{230,148}},
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
    end MetroscopiaCCGT_deSH_reverse;
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
