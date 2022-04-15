within MetroscopeModelingLibrary.Examples;
package Nuclear
  extends Modelica.Icons.ExamplesPackage;
  package MetroscopiaNPP
    extends Modelica.Icons.ExamplesPackage;
    model Metroscopia_NPP_reverse
      // Boundary Conditions
        // Steam generator
        input Real steam_generator_vapor_fraction(start = 0.99);
        input Real steam_generator_steam_P_out(start = 50, unit="bar", min=0, nominal=50) "barA";
        input Real steam_generator_thermal_power(start = 1880) "MWth";

        // Boundary conditions
        input Real cold_source_P_out(start = 3, unit="bar", min=0, nominal=5) "barA";
        input Real cold_source_T_out(start = 15, unit="degC", min=0, nominal=15) "degC";

      // Observables used for calibration
        // HP Control Valve
        input Units.Fraction HP_control_valve_opening(start=0.15); // HP_control_valve_Cvmax

        // HP turbines
        input Real HP_turbine_1_P_in(start=48.5, unit="bar", min=0, nominal=50) "barA"; // HP_control_valve_Cv
        input Real HP_turbines_ext_P(start=31, unit="bar", min=0, nominal=50) "barA"; // HP_turbine_1_Cst
        input Real HP_turbine_2_P_out(start=19.4, unit="bar", min=0, nominal=50) "barA"; // HP_turbine_2_Cst

        // Superheater Control Valve
        input Units.Fraction superheater_control_valve_opening(start=0.9); // superheater_control_valve_Cvmax

        // Superheater
        input Real superheater_hot_P_in(start=41, unit="bar", min=0, nominal=50) "barA"; // superheater_control_valve_Cv
        input Real superheater_drains_P_out(start=40, unit="bar", min=0, nominal=50) "barA"; // superheater_Kfr_hot
        input Real superheated_steam_T_out(start=228) "°C"; // superheater_Kth

        // LP turbines
        input Real LP_turbines_ext_P(start=5, unit="bar", min=0, nominal=5) "barA"; // LP_turbine_1_Cst

        // Condenser
        input Real cold_source_Qv_out(start = -50) "m3/s";
        input Real condenser_P_in(start=69.8, unit="mbar", min=0, nominal=70) "mbar"; // LP_turbine_2_Cst
        input Real cold_sink_P_in(start=2, unit="bar", min=0, nominal=5) "barA"; // condenser_Kfr_cold

        // Generator
        input Real generator_W_elec(start=570) "MW"; // HP_LP_turbines_eta_is

        // LP pump
        input Real LP_pump_P_out(start=7, unit="bar", min=0, nominal=70) "barA"; // LP_pump_a3
        input Real LP_pump_T_out(start=39, unit="degC", min=0, nominal=20) "degC"; // LP_pump_b3

        // LP Reheater
        input Real LP_reheater_P_cold_out(start=6, min=0, nominal=50) "bar"; // LP_reheater_Kfr_cold
        input Real LP_reheater_T_cold_out(start=70, min=0, nominal=100) "degC"; // LP_reheater_Kth

        // LP reheater drains Control Valve
        input Units.Fraction LP_reheater_drains_control_valve_opening(start=0.15); // LP_control_valve_Cvmax
        input Real LP_reheater_drains_control_valve_P_out(start=4, min=0, nominal=5) "bar";

        // Flash tank : none

        // HP pump
        input Real HP_pump_P_out(start=59, unit="bar", min=0, nominal=70) "barA"; // LP_pump_a3
        input Real HP_pump_T_out(start=74.4, unit="degC", min=0, nominal=20) "degC"; // LP_pump_b3

        // HP Reheater
        input Real HP_reheater_P_cold_out(start=58, min=0, nominal=50) "bar"; // LP_reheater_Kfr_cold
        input Real HP_reheater_T_cold_out(start=210, min=0, nominal=100) "degC"; // LP_reheater_Kth
        input Real HP_reheater_T_drains(start=70, min=0, nominal=100) "degC"; // LP_reheater_Kth

        // HP reheater drains Control Valve
        input Units.Fraction HP_reheater_drains_control_valve_opening(start=0.15); // HP_control_valve_Cvmax
        input Real HP_reheater_drains_control_valve_P_out(start=29, min=0, nominal=50) "bar";

      // Calibrated parameters
        // HP turbines inlet control valve
        output Units.Cv HP_control_valve_Cvmax; // HP_control_valve_opening
        output Units.Cv HP_control_valve_Cv; // HP_turbine_1_P_in

        // HP Turbines
        output Units.Cst HP_turbine_1_Cst; // HP_turbines_ext_P
        output Units.Cst HP_turbine_2_Cst; // HP_turbine_2_P_out

        // Superheater inlet control valve
        output Units.Cv superheater_control_valve_Cvmax; // superheater_control_valve_opening
        output Units.Cv superheater_control_valve_Cv; // superheater_hot_P_in

        // Superheater
        output Units.FrictionCoefficient superheater_Kfr_hot; // superheater_drains_P_out
        output Units.HeatExchangeCoefficient superheater_Kth; // superheated_steam_T_out

        // LP Turbines
        output Units.Cst LP_turbine_1_Cst; // LP_turbines_ext_P
        output Units.Cst LP_turbine_2_Cst; // LP_turbine_2_P_out = condenser_P_in

        output Units.Yield HP_LP_turbines_eta_is; // generator_W_elec

        // Condenser
        output Units.HeatExchangeCoefficient condenser_Kth;
        output Units.FrictionCoefficient condenser_Kfr_cold; // cold_sink_P_in

        // LP pump
        output Real LP_pump_a3; // LP_pump_P_out
        output Real LP_pump_b3; // LP_pump_T_out

        // LP Reheater
        output Units.HeatExchangeCoefficient LP_reheater_Kth;
        output Units.FrictionCoefficient LP_reheater_Kfr_cold;

        // LP Reheater drains control valve
        output Units.Cv LP_reheater_drains_control_valve_Cvmax; // LP_control_valve_opening
        output Units.Cv LP_reheater_drains_control_valve_Cv; //

        // Flash tank : none

        // LP pump
        output Real HP_pump_a3; // HP_pump_P_out
        output Real HP_pump_b3; // HP_pump_T_out

        // HP Reheater
        output Units.HeatExchangeCoefficient HP_reheater_Kth_cond;
        output Units.HeatExchangeCoefficient HP_reheater_Kth_subc;
        output Units.FrictionCoefficient HP_reheater_Kfr_cold;

        // HP Reheater drains control valve
        output Units.Cv HP_reheater_drains_control_valve_Cvmax; // HP_control_valve_opening
        output Units.Cv HP_reheater_drains_control_valve_Cv; //

      // Components
        // Steam Generator
        WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-196,-116},{-152,-24}})));
        WaterSteam.BoundaryConditions.Sink blow_down_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=180,origin={-190,-132})));

        // HP
          // HP Control Valve
          WaterSteam.Pipes.ControlValve HP_control_valve annotation (Placement(transformation(extent={{-136,70},{-126,82}})));
          Sensors.Other.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,86},{-126,96}})));

          // HP Turbines
          Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_P_in_sensor annotation (Placement(transformation(extent={{-106,66.1818},{-94,78.1818}})));
          WaterSteam.Machines.StodolaTurbine HP_turbine_1 annotation (Placement(transformation(extent={{-80,64.1818},{-62,80.1818}})));
          WaterSteam.Machines.StodolaTurbine HP_turbine_2 annotation (Placement(transformation(extent={{-8,64.1818},{10,80.1818}})));
          WaterSteam.Pipes.SteamExtractionSplitter HP_turbines_ext annotation (Placement(transformation(extent={{-46,62.1818},{-26,80.1818}})));
          Sensors.WaterSteam.WaterPressureSensor HP_turbines_ext_P_sensor annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=270,
            origin={-36,53})));
          Sensors.WaterSteam.WaterPressureSensor HP_turbine_2_P_out_sensor annotation (Placement(transformation(extent={{20,66.1818},{32,78.1818}})));

        // Generator
        Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{362,158},{382,178}})));
        Power.Machines.Generator generator annotation (Placement(transformation(extent={{308,156},{348,180}})));
        Sensors.Power.PowerSensor generator_W_elec_sensor annotation (Placement(transformation(extent={{348,162},{360,174}})));

        // Temporary components

      // Unclassified components
      WaterSteam.Volumes.SteamDryer steam_dryer annotation (Placement(transformation(extent={{48,60.1818},{64,78.1818}})));
      WaterSteam.HeatExchangers.SuperHeater superheater annotation (Placement(transformation(extent={{56,104},{88,120}})));
      WaterSteam.BoundaryConditions.Sink superheater_vent_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={88,86.182})));
      Sensors.WaterSteam.WaterPressureSensor superheater_drains_P_out_sensor annotation (Placement(transformation(extent={{100,106},{112,118}})));
      Sensors.WaterSteam.WaterTemperatureSensor superheated_steam_T_out_sensor annotation (Placement(transformation(extent={{88,124},{100,136}})));
      WaterSteam.Pipes.ControlValve superheater_control_valve annotation (Placement(transformation(extent={{-136,110},{-126,122}})));
      Sensors.Other.OpeningSensor superheater_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,132},{-126,142}})));
      Sensors.WaterSteam.WaterPressureSensor superheater_hot_P_in_sensor annotation (Placement(transformation(extent={{-106,106.182},{-94,118.182}})));
      WaterSteam.Machines.StodolaTurbine LP_turbine_1 annotation (Placement(transformation(extent={{152,122.182},{170,138.182}})));
      WaterSteam.Machines.StodolaTurbine LP_turbine_2 annotation (Placement(transformation(extent={{224,122.182},{242,138.182}})));
      Sensors.WaterSteam.WaterPressureSensor LP_turbines_ext_P_sensor annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=270,
            origin={196,111})));
      Sensors.WaterSteam.WaterPressureSensor condenser_P_in_sensor annotation (Placement(transformation(extent={{286,124},{298,136}})));
      WaterSteam.HeatExchangers.Condenser condenser annotation (Placement(transformation(extent={{379,54},{405,76}})));
      WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={440,63.7778})));
      Sensors.WaterSteam.WaterPressureSensor cold_sink_P_in_sensor annotation (Placement(transformation(extent={{418,57.7778},{430,69.7778}})));
      WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{328,59.7778},{348,79.7778}})));
      Sensors.WaterSteam.WaterFlowSensor cold_source_Qv_out_sensor annotation (Placement(transformation(extent={{352,62.7778},{366,76.7778}})));
        WaterSteam.Machines.Pump LP_pump annotation (Placement(transformation(extent={{380,-78},{364,-62}})));
        Power.BoundaryConditions.Source LP_pump_Wm_source annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={372,-46})));
        Sensors.WaterSteam.WaterTemperatureSensor LP_pump_T_out_sensor annotation (Placement(transformation(extent={{350,-77},{336,-63}})));
        Sensors.WaterSteam.WaterPressureSensor LP_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}},     origin={315,-70})));
      WaterSteam.HeatExchangers.DryReheater LP_reheater
        annotation (Placement(transformation(extent={{284,-78},{252,-62}})));
      Sensors.WaterSteam.WaterTemperatureSensor LP_reheater_T_cold_out_sensor annotation (Placement(transformation(extent={{220,-77},{206,-63}})));
      Sensors.WaterSteam.WaterPressureSensor LP_reheater_P_cold_out_sensor annotation (Placement(transformation(extent={{242,-77},{228,-63}})));
      WaterSteam.Pipes.SteamExtractionSplitter LP_turbines_ext annotation (Placement(transformation(extent={{186,120},{206,138}})));
      WaterSteam.Pipes.Pipe flash_tank_inlet_pipe annotation (Placement(transformation(extent={{200,-80},{180,-60}})));
      WaterSteam.Pipes.PressureCut
                            steam_dryer_liq_out_pipe annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={142,-8})));
      WaterSteam.Pipes.Pipe flash_tank_outlet_pipe
                                                  annotation (Placement(transformation(extent={{100,-80},{80,-60}})));
        WaterSteam.Machines.Pump HP_pump annotation (Placement(transformation(extent={{66,-78},{50,-62}})));
        Power.BoundaryConditions.Source HP_pump_Wm_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={58,-46})));
        Sensors.WaterSteam.WaterTemperatureSensor HP_pump_T_out_sensor annotation (Placement(transformation(extent={{36,-77},{22,-63}})));
        Sensors.WaterSteam.WaterPressureSensor HP_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={1,-70})));
      WaterSteam.HeatExchangers.Reheater HP_reheater(Q_cold_0=1500, Q_hot_0=50) annotation (Placement(transformation(extent={{-20,-78},{-52,-62}})));
      Sensors.WaterSteam.WaterTemperatureSensor HP_reheater_T_cold_out_sensor annotation (Placement(transformation(extent={{-80,-77},{-94,-63}})));
      Sensors.WaterSteam.WaterPressureSensor HP_reheater_P_cold_out_sensor annotation (Placement(transformation(extent={{-60,-77},{-74,-63}})));
        Sensors.WaterSteam.WaterTemperatureSensor HP_reheater_T_drains_sensor annotation (Placement(transformation(
            extent={{7,-7},{-7,7}},
            rotation=90,
            origin={-36,-98})));
      WaterSteam.Pipes.PressureCut
                            superheater_drains_pipe annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={122,32})));
      Sensors.WaterSteam.WaterFlowSensor steam_generator_Q_in_sensor annotation (Placement(transformation(extent={{-100,-77},{-114,-63}})));
          WaterSteam.Pipes.ControlValve HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{6,-124},{16,-112}})));
          Sensors.Other.OpeningSensor HP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{6,-108},{16,-98}})));
      Sensors.WaterSteam.WaterPressureSensor HP_reheater_drains_control_valve_P_out_sensor annotation (Placement(transformation(extent={{24,-127.818},{36,-115.818}})));
      WaterSteam.Pipes.PressureCut
                            HP_reheater_drains_pipe
                                                  annotation (Placement(transformation(extent={{64,-132},{84,-112}})));
          WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve annotation (Placement(transformation(extent={{288,-122},{298,-110}})));
          Sensors.Other.OpeningSensor LP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{288,-106},{298,-96}})));
      Sensors.WaterSteam.WaterPressureSensor LP_reheater_drains_control_valve_P_out_sensor annotation (Placement(transformation(extent={{306,-125.818},{318,-113.818}})));
      WaterSteam.Pipes.PressureCut
                            LP_reheater_drains_pipe annotation (Placement(transformation(extent={{346,-130},{366,-110}})));
    equation
      // ----- Boundary Conditions ------
      steam_generator.vapor_fraction = steam_generator_vapor_fraction;
      //steam_generator.thermal_power = steam_generator_thermal_power * 1e6;
      steam_generator.steam_pressure = steam_generator_steam_P_out * 1e5;

      cold_source.P_out = cold_source_P_out * 1e5;
      cold_source.T_out = cold_source_T_out + 273.15;

      // Temporary components
      /*
  temp_feedwater_source.Q_out = -2000;
  temp_feedwater_source.P_out = 58e5;
  temp_feedwater_source.T_out = 273.15 + 225;
  */
      // ----- Components ------
      // SteamGenerator
        // Observable used for calibration
        //steam_generator_P_in_sensor.P_barA = steam_generator_P_in;

        // Purge fixed parameters
        //steam_generator.Q_purge = 1e-2;
        steam_generator.P_purge = 50e5;
        steam_generator_Q_in_sensor.Q = 1500;

      // HP systems
        // HP turbines inlet control valve
          // Observables used for calibration (inlet)
          HP_turbine_1_P_in_sensor.P_barA = HP_turbine_1_P_in;
          HP_control_valve_opening_sensor.Opening = HP_control_valve_opening;

          // Calibrated parameters
          HP_control_valve.Cvmax = HP_control_valve_Cvmax;
          HP_control_valve.Cv = HP_control_valve_Cv;

        // HP Turbine 1
          // Calibrated parameters
          HP_turbine_1.Cst = HP_turbine_1_Cst;
          HP_turbine_1.eta_is = HP_LP_turbines_eta_is;

          // Hypothesis : no nozzle
          HP_turbine_1.eta_nz = 1;
          HP_turbine_1.area_nz = 1;

          // HP extraction
            // Fixed parameter
            HP_turbines_ext.alpha = 1;

            // Observable used for calibration
            HP_turbines_ext_P_sensor.P_barA = HP_turbines_ext_P;

        // HP Turbine 2
          // Calibrated parameters
          HP_turbine_2.Cst = HP_turbine_2_Cst;
          HP_turbine_2.eta_is = HP_LP_turbines_eta_is;

          // Hypothesis : no nozzle
          HP_turbine_2.eta_nz = 1;
          HP_turbine_2.area_nz = 1;

          // Observable used for calibration
          HP_turbine_2_P_out_sensor.P_barA = HP_turbine_2_P_out;

      // Steam dryer
      steam_dryer.x_steam_out = 0.99;

      // Superheater
        // Superheater control valve
          // Observables used for calibration (inlet)
          superheater_hot_P_in_sensor.P_barA = superheater_hot_P_in;
          superheater_control_valve_opening_sensor.Opening = superheater_control_valve_opening;

          // Calibrated parameters
          superheater_control_valve.Cvmax = superheater_control_valve_Cvmax;
          superheater_control_valve.Cv = superheater_control_valve_Cv;

        // Fixed parameters
        superheater.S = 100;
        superheater.Kfr_cold = 0;
        superheater.Q_vent = 1;

        // Observables used for calibration
        superheater_drains_P_out_sensor.P_barA = superheater_drains_P_out;
        superheated_steam_T_out_sensor.T_degC = superheated_steam_T_out;

        // Calibrated parameters
        superheater.Kfr_hot = superheater_Kfr_hot;
        superheater.Kth = superheater_Kth;

      // LP systems
        // LP Turbine 1
          // Observable used for calibration
          //LP_turbine_1_P_in_sensor.P_barA = LP_turbine_1_P_in;

          // Calibrated parameters
          LP_turbine_1.Cst = LP_turbine_1_Cst;
          LP_turbine_1.eta_is = HP_LP_turbines_eta_is;

          // Hypothesis : no nozzle
          LP_turbine_1.eta_nz = 1;
          LP_turbine_1.area_nz = 1;

          // LP Extraction
            // Fixed parameter
            LP_turbines_ext.alpha = 1;

            // Observable used for calibration
            LP_turbines_ext_P_sensor.P_barA = LP_turbines_ext_P;

        // LP Turbine 2
          // Calibrated parameters
          LP_turbine_2.Cst = LP_turbine_2_Cst;
          LP_turbine_2.eta_is = HP_LP_turbines_eta_is;

          // Hypothesis : no nozzle
          LP_turbine_2.eta_nz = 1;
          LP_turbine_2.area_nz = 1;

      // Generator
        // Observable used for calibration
        generator_W_elec_sensor.W_MW = generator_W_elec; // Calibrates STs_eta_is

        // Fixed parameter
        generator.eta = 0.99;

      // Condenser
        // Fixed parameters
        condenser.S = 100;
        condenser.water_height = 1;
        condenser.C_incond = 0;
        condenser.P_offset = 0;

        // Observable used for calibration
        cold_source_Qv_out_sensor.Qv_out = cold_source_Qv_out;
        condenser_P_in_sensor.P_mbar = condenser_P_in;
        cold_sink_P_in_sensor.P_barA = cold_sink_P_in;

        // Calibrated Parameters
        condenser.Kth = condenser_Kth;
        condenser.Kfr_cold = condenser_Kfr_cold;

      // LP pump
        // Fixed parameters
        LP_pump.VRotn = 4000;
        LP_pump.VRot = 3950;
        LP_pump.rm = 0.85;
        LP_pump.a1 = -78.0237;
        LP_pump.a2 = 0;
        LP_pump.b1 = 0;
        LP_pump.b2 = 0;
        LP_pump.rhmin = 0.2;

        // Calibrated parameters
        LP_pump.a3 = LP_pump_a3;
        LP_pump.b3 = LP_pump_b3;

        // Inputs for calibration
        LP_pump_T_out_sensor.T_degC = LP_pump_T_out;
        LP_pump_P_out_sensor.P_barA = LP_pump_P_out;

      // LP Reheater
        // Component parameters
        LP_reheater.S_condensing = 100;
        LP_reheater.Kfr_hot = 0;

        // Observables for calibration
        LP_reheater_P_cold_out_sensor.P_barA = LP_reheater_P_cold_out;
        LP_reheater_T_cold_out_sensor.T_degC = LP_reheater_T_cold_out;

        // Calibrated parameters
        LP_reheater.Kth = LP_reheater_Kth;
        LP_reheater.Kfr_cold = LP_reheater_Kfr_cold;

        // Drains
          // Observables used for calibration
          LP_reheater_drains_control_valve_P_out_sensor.P_barA = LP_reheater_drains_control_valve_P_out;
          LP_reheater_drains_control_valve_opening_sensor.Opening = LP_reheater_drains_control_valve_opening;

          // Calibrated parameters
          LP_reheater_drains_control_valve.Cvmax = LP_reheater_drains_control_valve_Cvmax;
          LP_reheater_drains_control_valve.Cv = LP_reheater_drains_control_valve_Cv;

      // Flash tank
        // Inlet Pipe
        flash_tank_inlet_pipe.Kfr = 0;
        flash_tank_inlet_pipe.delta_z = 5;

        flash_tank_outlet_pipe.Kfr = 0;
        flash_tank_outlet_pipe.delta_z = -5;

      // HP pump
        // Fixed parameters
        HP_pump.VRotn = 4000;
        HP_pump.VRot = 3950;
        HP_pump.rm = 0.85;
        HP_pump.a1 = -78.0237;
        HP_pump.a2 = 0;
        HP_pump.b1 = 0;
        HP_pump.b2 = 0;
        HP_pump.rhmin = 0.2;

        // Calibrated parameters
        HP_pump.a3 = HP_pump_a3;
        HP_pump.b3 = HP_pump_b3;

        // Inputs for calibration
        HP_pump_T_out_sensor.T_degC = HP_pump_T_out;
        HP_pump_P_out_sensor.P_barA = HP_pump_P_out;

      // HP Reheater
        // Component parameters
        HP_reheater.S_tot = 100;
        HP_reheater.Kfr_hot = 0;
        HP_reheater.level = 0.3;

        // Observables for calibration
        HP_reheater_P_cold_out_sensor.P_barA = HP_reheater_P_cold_out;
        HP_reheater_T_cold_out_sensor.T_degC = HP_reheater_T_cold_out;
        HP_reheater_T_drains_sensor.T_degC = HP_reheater_T_drains;

        // Calibrated parameters
        HP_reheater.Kth_subc = HP_reheater_Kth_subc;
        HP_reheater.Kth_cond = HP_reheater_Kth_cond;
        HP_reheater.Kfr_cold = HP_reheater_Kfr_cold;

        // Drains
          // Observables used for calibration
          HP_reheater_drains_control_valve_P_out_sensor.P_barA = HP_reheater_drains_control_valve_P_out;
          HP_reheater_drains_control_valve_opening_sensor.Opening = HP_reheater_drains_control_valve_opening;

          // Calibrated parameters
          HP_reheater_drains_control_valve.Cvmax = HP_reheater_drains_control_valve_Cvmax;
          HP_reheater_drains_control_valve.Cv = HP_reheater_drains_control_valve_Cv;

      connect(steam_generator.purge_outlet, blow_down_sink.C_in) annotation (Line(points={{-174,-115.233},{-174,-132},{-185,-132}},
                                                                                                                       color={28,108,200}));
      connect(steam_generator.steam_outlet, HP_control_valve.C_in) annotation (Line(points={{-174,-24},{-174,72.1818},{-136,72.1818}},color={28,108,200}));
      connect(HP_control_valve.C_out, HP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{-126,72.1818},{-116,72.1818},{-116,72.1818},{-106,72.1818}},
                                                                                                                                                          color={28,108,200}));
      connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,80.9091},{-131,85.9}}, color={0,0,127}));
      connect(HP_turbine_1.C_out, HP_turbines_ext.C_in) annotation (Line(points={{-62,72.1818},{-46.6,72.1818}}, color={28,108,200}));
      connect(HP_turbines_ext.C_main_out, HP_turbine_2.C_in) annotation (Line(points={{-25.4,72.1818},{-8,72.1818}}, color={28,108,200}));
      connect(HP_turbines_ext.C_ext_out, HP_turbines_ext_P_sensor.C_in) annotation (Line(points={{-36,65.3818},{-36,60}}, color={28,108,200}));
      connect(powerSink.C_in,generator_W_elec_sensor. C_out) annotation (Line(points={{367,168},{359.88,168}},
                                                                                                  color={244,125,35}));
      connect(generator_W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{348,168},{342,168}},
                                                                                               color={244,125,35}));
      connect(HP_turbine_2.C_W_out,generator. C_in) annotation (Line(points={{10,78.9018},{18,78.9018},{18,168},{315.6,168}},
                                                                                                                       color={244,125,35}));
      connect(HP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{-62,78.9018},{-54,78.9018},{-54,168},{315.6,168}},
                                                                                                               color={244,125,35}));
      connect(HP_turbine_1.C_in, HP_turbine_1_P_in_sensor.C_out) annotation (Line(points={{-80,72.1818},{-94,72.1818}},                 color={28,108,200}));
      connect(HP_turbine_2_P_out_sensor.C_in, HP_turbine_2.C_out) annotation (Line(points={{20,72.1818},{10,72.1818}},
                                                                                                             color={28,108,200}));
      connect(steam_dryer.C_in, HP_turbine_2_P_out_sensor.C_out) annotation (Line(points={{48,71.6363},{40,71.6363},{40,72.1818},{32,72.1818}}, color={28,108,200}));
      connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,104},{72,71.6363},{64,71.6363}}, color={28,108,200}));
      connect(superheater.C_hot_out, superheater_drains_P_out_sensor.C_in) annotation (Line(points={{88,112},{100,112}}, color={28,108,200}));
      connect(superheated_steam_T_out_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,130},{71.8,130.182},{71.8,120}}, color={28,108,200}));
      connect(superheater_control_valve.C_in, HP_control_valve.C_in) annotation (Line(points={{-136,112.182},{-174,112.182},{-174,72.1818},{-136,72.1818}}, color={28,108,200}));
      connect(superheater_control_valve.Opening, superheater_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,120.909},{-131,131.9}}, color={0,0,127}));
      connect(superheater.C_hot_in, superheater_hot_P_in_sensor.C_out) annotation (Line(points={{56,112.2},{-35,112.2},{-35,112.182},{-94,112.182}}, color={28,108,200}));
      connect(superheater_hot_P_in_sensor.C_in, superheater_control_valve.C_out) annotation (Line(points={{-106,112.182},{-116,112.182},{-116,112.182},{-126,112.182}}, color={28,108,200}));
      connect(LP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{170,136.902},{188,136.902},{188,168},{315.6,168}},
                                                                                                               color={244,125,35}));
      connect(LP_turbine_2.C_W_out, generator.C_in) annotation (Line(points={{242,136.902},{262,136.902},{262,168},{315.6,168}}, color={244,125,35}));
      connect(LP_turbine_2.C_out, condenser_P_in_sensor.C_in) annotation (Line(points={{242,130.182},{264,130.182},{264,130},{286,130}}, color={28,108,200}));
      connect(condenser_P_in_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{298,130},{392,130},{392,76}}, color={28,108,200}));
      connect(cold_sink_P_in_sensor.C_out, cold_sink.C_in) annotation (Line(points={{430,63.7778},{435,63.7778}},                     color={28,108,200}));
      connect(condenser.C_cold_out, cold_sink_P_in_sensor.C_in) annotation (Line(points={{405,63.0444},{411.5,63.0444},{411.5,63.7778},{418,63.7778}}, color={28,108,200}));
      connect(superheater.C_vent, superheater_vent_sink.C_in) annotation (Line(points={{88,104.2},{88,91.182}}, color={28,108,200}));
      connect(superheated_steam_T_out_sensor.C_out, LP_turbine_1.C_in) annotation (Line(points={{100,130},{126,130},{126,130.182},{152,130.182}}, color={28,108,200}));
      connect(condenser.C_cold_in, cold_source_Qv_out_sensor.C_out) annotation (Line(points={{378.48,69.8889},{378.48,69.7778},{366,69.7778}}, color={28,108,200}));
      connect(cold_source_Qv_out_sensor.C_in, cold_source.C_out) annotation (Line(points={{352,69.7778},{343,69.7778}}, color={28,108,200}));
      connect(LP_pump.C_power,LP_pump_Wm_source. C_out) annotation (Line(points={{372,-61.36},{372,-50.8}}, color={244,125,35}));
      connect(LP_pump.C_out,LP_pump_T_out_sensor. C_in) annotation (Line(points={{364,-70},{350,-70}}, color={28,108,200}));
      connect(LP_pump_T_out_sensor.C_out,LP_pump_P_out_sensor. C_in) annotation (Line(points={{336,-70},{322,-70}}, color={28,108,200}));
      connect(condenser.C_hot_out, LP_pump.C_in) annotation (Line(points={{392,53.5111},{392,-70},{380,-70}}, color={28,108,200}));
      connect(LP_reheater.C_cold_out,LP_reheater_P_cold_out_sensor. C_in) annotation (Line(points={{252,-70},{242,-70}},
                                                                                                        color={28,108,200}));
      connect(LP_reheater_P_cold_out_sensor.C_out,LP_reheater_T_cold_out_sensor. C_in) annotation (Line(points={{228,-70},{224,-70},{224,-70.5},{220,-70.5},{220,-70}},
                                                                                                          color={28,108,200}));
      connect(LP_pump_P_out_sensor.C_out,LP_reheater. C_cold_in) annotation (Line(points={{308,-70},{284.2,-70}},                     color={28,108,200}));
      connect(LP_turbines_ext_P_sensor.C_out,LP_reheater. C_hot_in) annotation (Line(points={{196,104},{196,44},{268,44},{268,-62}}, color={28,108,200}));
      connect(LP_turbines_ext_P_sensor.C_in, LP_turbines_ext.C_ext_out) annotation (Line(points={{196,118},{196,123.2}}, color={28,108,200}));
      connect(LP_turbine_1.C_out, LP_turbines_ext.C_in) annotation (Line(points={{170,130.182},{177.7,130.182},{177.7,130},{185.4,130}}, color={28,108,200}));
      connect(LP_turbine_2.C_in, LP_turbines_ext.C_main_out) annotation (Line(points={{224,130.182},{215.3,130.182},{215.3,130},{206.6,130}}, color={28,108,200}));
      connect(flash_tank_inlet_pipe.C_in, LP_reheater_T_cold_out_sensor.C_out) annotation (Line(points={{200,-70},{206,-70}}, color={28,108,200}));
      connect(steam_dryer.C_hot_liquid, steam_dryer_liq_out_pipe.C_in) annotation (Line(points={{64,65.0909},{64,64},{142,64},{142,2}},  color={28,108,200}));
      connect(flash_tank_inlet_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{180,-70},{100,-70}}, color={28,108,200}));
      connect(steam_dryer_liq_out_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{142,-18},{142,-70},{100,-70}},
                                                                                                                                  color={28,108,200}));
      connect(HP_pump.C_power, HP_pump_Wm_source.C_out) annotation (Line(points={{58,-61.36},{58,-50.8}}, color={244,125,35}));
      connect(HP_pump.C_out, HP_pump_T_out_sensor.C_in) annotation (Line(points={{50,-70},{36,-70}}, color={28,108,200}));
      connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in) annotation (Line(points={{22,-70},{8,-70}},  color={28,108,200}));
      connect(HP_pump.C_in, flash_tank_outlet_pipe.C_out) annotation (Line(points={{66,-70},{80,-70}}, color={28,108,200}));
      connect(HP_pump_P_out_sensor.C_out, HP_reheater.C_cold_in) annotation (Line(points={{-6,-70},{-19.8,-70}}, color={28,108,200}));
      connect(HP_reheater.C_cold_out, HP_reheater_P_cold_out_sensor.C_in) annotation (Line(points={{-52,-70},{-60,-70}}, color={28,108,200}));
      connect(HP_reheater_P_cold_out_sensor.C_out, HP_reheater_T_cold_out_sensor.C_in) annotation (Line(points={{-74,-70},{-80,-70}},                             color={28,108,200}));
      connect(HP_reheater_T_drains_sensor.C_in, HP_reheater.C_hot_out) annotation (Line(points={{-36,-91},{-36,-78}}, color={28,108,200}));
      connect(HP_turbines_ext_P_sensor.C_out, HP_reheater.C_hot_in) annotation (Line(points={{-36,46},{-36,-62}}, color={28,108,200}));
      connect(superheater_drains_P_out_sensor.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{112,112},{122,112},{122,42}}, color={28,108,200}));
      connect(superheater_drains_pipe.C_out, HP_reheater.C_hot_in) annotation (Line(points={{122,22},{122,0},{-36,0},{-36,-62}}, color={28,108,200}));
      connect(HP_reheater_T_cold_out_sensor.C_out, steam_generator_Q_in_sensor.C_in) annotation (Line(points={{-94,-70},{-100,-70}},                    color={28,108,200}));
      connect(steam_generator.feedwater_inlet, steam_generator_Q_in_sensor.C_out) annotation (Line(points={{-163,-70},{-114,-70}},                           color={28,108,200}));
      connect(HP_reheater_drains_control_valve.Opening, HP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{11,-113.091},{11,-108.1}}, color={0,0,127}));
      connect(HP_reheater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-36,-105},{-36,-121.818},{6,-121.818}}, color={28,108,200}));
      connect(HP_reheater_drains_control_valve.C_out, HP_reheater_drains_control_valve_P_out_sensor.C_in) annotation (Line(points={{16,-121.818},{20,-121.818},{20,-121.818},{24,-121.818}}, color={28,108,200}));
      connect(HP_reheater_drains_control_valve_P_out_sensor.C_out, HP_reheater_drains_pipe.C_in) annotation (Line(points={{36,-121.818},{36,-122},{64,-122}}, color={28,108,200}));
      connect(HP_reheater_drains_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{84,-122},{142,-122},{142,-70},{100,-70}}, color={28,108,200}));
      connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{293,-111.091},{293,-106.1}}, color={0,0,127}));
      connect(LP_reheater_drains_control_valve.C_out, LP_reheater_drains_control_valve_P_out_sensor.C_in) annotation (Line(points={{298,-119.818},{302,-119.818},{302,-119.818},{306,-119.818}}, color={28,108,200}));
      connect(LP_reheater_drains_control_valve_P_out_sensor.C_out, LP_reheater_drains_pipe.C_in) annotation (Line(points={{318,-119.818},{318,-120},{346,-120}}, color={28,108,200}));
      connect(LP_reheater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{268,-78},{268,-119.818},{288,-119.818}}, color={28,108,200}));
      connect(LP_reheater_drains_pipe.C_out, condenser.C_hot_in) annotation (Line(points={{366,-120},{460,-120},{460,100},{392,100},{392,76}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{460,200}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{460,200}}), graphics={Rectangle(
              extent={{120,-54},{164,-82}},
              lineColor={28,108,200},
              lineThickness=1,
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid), Rectangle(
              extent={{120,-40},{164,-54}},
              lineColor={28,108,200},
              lineThickness=1,
              fillColor={170,213,255},
              fillPattern=FillPattern.Solid)}));
    end Metroscopia_NPP_reverse;
  end MetroscopiaNPP;

  model TurbineLine_direct
    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=6.05e6);
    input Units.SpecificEnthalpy source_h(start=2.78e6);
    input Units.OutletMassFlowRate source_Q(start=-1000);

    // Component parameters
    // Turbines
    parameter Units.Cst ST1_Cst = 14820;
    parameter Units.Cst ST2_Cst = 2941;
    parameter Units.Cst ST3_Cst = 1362;

    parameter Units.Yield STs_eta_is = 0.847;
    parameter Units.Yield STs_eta_nz = 1;
    parameter Units.Area STs_area_nz = 1;

    // Extraction splitters
    parameter Units.Fraction ST1_ext_alpha = 0.973;
    parameter Units.Fraction ST2_ext_alpha = 0.964;
    parameter Units.Fraction ST3_ext_alpha = 1;

    // Generator
    parameter Units.Yield generator_eta = 0.99;

    // Components
    // Boundary conditions
    WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
    WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{118,-10},{138,10}})));

    WaterSteam.BoundaryConditions.Sink ST1_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-66,-50})));
    WaterSteam.BoundaryConditions.Sink ST2_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={8,-50})));
    WaterSteam.BoundaryConditions.Sink ST3_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={84,-50})));

    // Turbines
    WaterSteam.Machines.StodolaTurbine ST1 annotation (Placement(transformation(extent={{-112,-10},{-92,10}})));
    WaterSteam.Machines.StodolaTurbine ST2 annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    WaterSteam.Machines.StodolaTurbine ST3 annotation (Placement(transformation(extent={{38,-10},{58,10}})));

    // Extractions
    WaterSteam.Pipes.SteamExtractionSplitter ST1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
    WaterSteam.Pipes.SteamExtractionSplitter ST2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
    WaterSteam.Pipes.SteamExtractionSplitter ST3_ext annotation (Placement(transformation(extent={{74,-10},{94,8}})));

    // Electricity
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{118,30},{138,50}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{78,28},{118,52}})));
  equation
    // Boundary conditions
    source.P_out = source_P;
    source.h_out = source_h;
    source.Q_out = source_Q;

    ST1_ext_sink.Q_in = source_Q/6;
    ST2_ext_sink.Q_in = source_Q/6;
    ST3_ext_sink.Q_in = source_Q/6;

    // Turbine 1
    ST1.Cst = ST1_Cst;
    ST1.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST1.eta_nz = STs_eta_nz;
    ST1.area_nz = STs_area_nz;

    // Extraction 1
    ST1_ext.alpha = ST1_ext_alpha;

    // Turbine 2
    ST2.Cst = ST2_Cst;
    ST2.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST2.eta_nz = STs_eta_nz;
    ST2.area_nz = STs_area_nz;

    // Extraction 2
    ST2_ext.alpha = ST2_ext_alpha;

    // Turbine 3
    ST3.Cst = ST3_Cst;
    ST3.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST3.eta_nz = STs_eta_nz;
    ST3.area_nz = STs_area_nz;

    // Extraction 3
    ST3_ext.alpha = ST3_ext_alpha;

    // Generator
    generator.eta = generator_eta;

    connect(ST1.C_in, source.C_out) annotation (Line(points={{-112,0},{-125,0}}, color={28,108,200}));
    connect(ST1.C_out, ST1_ext.C_in) annotation (Line(points={{-92,0},{-76.6,0}},  color={28,108,200}));
    connect(ST1_ext.C_main_out, ST2.C_in) annotation (Line(points={{-55.4,0},{-40,0}}, color={28,108,200}));
    connect(ST2.C_out, ST2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0}},  color={28,108,200}));
    connect(ST2_ext.C_main_out, ST3.C_in) annotation (Line(points={{18.6,0},{38,0}}, color={28,108,200}));
    connect(ST3.C_out, ST3_ext.C_in) annotation (Line(points={{58,0},{73.4,0}}, color={28,108,200}));
    connect(ST1_ext.C_ext_out, ST1_ext_sink.C_in) annotation (Line(points={{-66,-6.8},{-66,-45}}, color={28,108,200}));
    connect(ST3_ext.C_ext_out, ST3_ext_sink.C_in) annotation (Line(points={{84,-6.8},{84,-45}}, color={28,108,200}));
    connect(ST2_ext.C_ext_out, ST2_ext_sink.C_in) annotation (Line(points={{8,-6.8},{8,-45}},                   color={28,108,200}));
    connect(powerSink.C_in, generator.C_out) annotation (Line(points={{123,40},{112,40}}, color={244,125,35}));
    connect(ST3.C_W_out, generator.C_in) annotation (Line(points={{58,8.4},{68,8.4},{68,40},{85.6,40}},  color={244,125,35}));
    connect(ST2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{85.6,40}},  color={244,125,35}));
    connect(ST1.C_W_out, generator.C_in) annotation (Line(points={{-92,8.4},{-84,8.4},{-84,40},{85.6,40}},   color={244,125,35}));
    connect(ST3_ext.C_main_out, sink.C_in) annotation (Line(points={{94.6,0},{123,0}}, color={28,108,200}));
    annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
                                 Polygon(
            points={{-104,62},{-104,42},{-104,-38},{-104,-58},{-84,-64},{76,-98},{96,-98},{96,-78},{96,79.539},{96,102},{76,102},{-84,70},{-104,62}},
            lineColor={63,81,181},
            lineThickness=0.5,
            smooth=Smooth.Bezier),
                                 Polygon(
            points={{-96,60},{-96,42},{-96,-38},{-96,-52},{-78,-58},{68,-88},{88,-92},{88,-70},{88,72},{88,94},{68,92},{-76,64},{-96,60}},
            lineThickness=0.5,
            smooth=Smooth.Bezier,
            fillColor={207,211,237},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Line(
            points={{62,88},{62,-84}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{18,80},{18,-76}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{-24,70},{-24,-66}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{-64,62},{-64,-56}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Rectangle(
            extent={{70,4},{-80,0}},
            lineThickness=0.5,
            fillColor={157,166,218},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None)}));
  end TurbineLine_direct;

  model TurbineLine_reverse
    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=6.05e6);
    input Units.SpecificEnthalpy source_h(start=2.78e6);
    input Units.OutletMassFlowRate source_Q(start=-1000);

    // Hypothesis on component parameters
    // Turbines
    parameter Units.Yield STs_eta_nz = 1;
    parameter Units.Area STs_area_nz = 1;

    // Extraction splitters
    parameter Units.Fraction ST1_ext_alpha = 0.973;
    parameter Units.Fraction ST2_ext_alpha = 0.964;
    parameter Units.Fraction ST3_ext_alpha = 1;

    // Generator
    parameter Units.Yield generator_eta = 0.99;

    // Calibrated parameters
    // Turbines
    output Units.Cst ST1_Cst;
    output Units.Cst ST2_Cst;
    output Units.Cst ST3_Cst;

    output Units.Yield STs_eta_is;

    // Components
    // Boundary conditions
    WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
    WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{122,-10},{142,10}})));

    WaterSteam.BoundaryConditions.Sink ST1_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-66,-50})));
    WaterSteam.BoundaryConditions.Sink ST2_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={8,-50})));
    WaterSteam.BoundaryConditions.Sink ST3_ext_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={84,-50})));

    // Turbines
    WaterSteam.Machines.StodolaTurbine ST1 annotation (Placement(transformation(extent={{-112,-10},{-92,10}})));
    WaterSteam.Machines.StodolaTurbine ST2 annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    WaterSteam.Machines.StodolaTurbine ST3 annotation (Placement(transformation(extent={{38,-10},{58,10}})));

    // Extractions
    WaterSteam.Pipes.SteamExtractionSplitter ST1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
    WaterSteam.Pipes.SteamExtractionSplitter ST2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
    WaterSteam.Pipes.SteamExtractionSplitter ST3_ext annotation (Placement(transformation(extent={{74,-10},{94,8}})));

    // Electricity
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{122,30},{142,50}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{68,28},{108,52}})));
    Sensors.WaterSteam.WaterPressureSensor ST1_ext_P_sensor annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=270,
          origin={-66,-25})));
    Sensors.WaterSteam.WaterPressureSensor ST3_ext_P_sensor annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=270,
          origin={84,-25})));
    Sensors.WaterSteam.WaterPressureSensor ST2_ext_P_sensor annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=270,
          origin={8,-25})));
    Sensors.Power.PowerSensor W_tot_sensor annotation (Placement(transformation(extent={{108,34},{120,46}})));
  equation
    // Boundary conditions
    source.P_out = source_P;
    source.h_out = source_h;
    source.Q_out = source_Q;

    ST1_ext_sink.Q_in = source_Q/6;
    ST2_ext_sink.Q_in = source_Q/6;
    ST3_ext_sink.Q_in = source_Q/6;

    // Turbine 1
    ST1.Cst = ST1_Cst;
    ST1.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST1.eta_nz = STs_eta_nz;
    ST1.area_nz = STs_area_nz;

    // Extraction 1
    ST1_ext_P_sensor.P_barA = 53; // Calibrates ST1_Cst
    ST1_ext.alpha = ST1_ext_alpha;

    // Turbine 2
    ST2.Cst = ST2_Cst;
    ST2.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST2.eta_nz = STs_eta_nz;
    ST2.area_nz = STs_area_nz;

    // Extraction 2
    ST2_ext_P_sensor.P_barA = 51; // Calibrates ST2_Cst
    ST2_ext.alpha = ST2_ext_alpha;

    // Turbine 3
    ST3.Cst = ST3_Cst;
    ST3.eta_is = STs_eta_is;

    // Hypothesis : no nozzle
    ST3.eta_nz = STs_eta_nz;
    ST3.area_nz = STs_area_nz;

    // Extraction 3
    ST3_ext_P_sensor.P_barA = 50; // Calibrates ST3_Cst
    ST3_ext.alpha = ST3_ext_alpha;

    // Generator
    W_tot_sensor.W_MW = 300; // Calibrates STs_eta_is
    // Hypothesis
    generator.eta = generator_eta;

    connect(ST1.C_in, source.C_out) annotation (Line(points={{-112,0},{-125,0}}, color={28,108,200}));
    connect(ST1.C_out, ST1_ext.C_in) annotation (Line(points={{-92,0},{-76.6,0}},  color={28,108,200}));
    connect(ST1_ext.C_main_out, ST2.C_in) annotation (Line(points={{-55.4,0},{-40,0}}, color={28,108,200}));
    connect(ST2.C_out, ST2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0}},  color={28,108,200}));
    connect(ST2_ext.C_main_out, ST3.C_in) annotation (Line(points={{18.6,0},{38,0}}, color={28,108,200}));
    connect(ST3.C_out, ST3_ext.C_in) annotation (Line(points={{58,0},{73.4,0}}, color={28,108,200}));
    connect(ST3.C_W_out, generator.C_in) annotation (Line(points={{58,8.4},{68,8.4},{68,40},{75.6,40}},  color={244,125,35}));
    connect(ST2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{75.6,40}},  color={244,125,35}));
    connect(ST1.C_W_out, generator.C_in) annotation (Line(points={{-92,8.4},{-84,8.4},{-84,40},{75.6,40}},   color={244,125,35}));
    connect(ST3_ext.C_main_out, sink.C_in) annotation (Line(points={{94.6,0},{127,0}}, color={28,108,200}));
    connect(ST1_ext.C_ext_out, ST1_ext_P_sensor.C_in) annotation (Line(points={{-66,-6.8},{-66,-18}}, color={28,108,200}));
    connect(ST1_ext_P_sensor.C_out, ST1_ext_sink.C_in) annotation (Line(points={{-66,-32},{-66,-45}}, color={28,108,200}));
    connect(ST2_ext.C_ext_out, ST2_ext_P_sensor.C_in) annotation (Line(points={{8,-6.8},{8,-18}}, color={28,108,200}));
    connect(ST2_ext_P_sensor.C_out, ST2_ext_sink.C_in) annotation (Line(points={{8,-32},{8,-45}}, color={28,108,200}));
    connect(ST3_ext.C_ext_out, ST3_ext_P_sensor.C_in) annotation (Line(points={{84,-6.8},{84,-18}}, color={28,108,200}));
    connect(ST3_ext_P_sensor.C_out, ST3_ext_sink.C_in) annotation (Line(points={{84,-32},{84,-45}}, color={28,108,200}));
    connect(powerSink.C_in, W_tot_sensor.C_out) annotation (Line(points={{127,40},{119.88,40}}, color={244,125,35}));
    connect(W_tot_sensor.C_in, generator.C_out) annotation (Line(points={{108,40},{102,40}}, color={244,125,35}));
    annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
                                 Polygon(
            points={{-104,62},{-104,42},{-104,-38},{-104,-58},{-84,-64},{76,-98},{96,-98},{96,-78},{96,79.539},{96,102},{76,102},{-84,70},{-104,62}},
            lineColor={63,81,181},
            lineThickness=0.5,
            smooth=Smooth.Bezier),
                                 Polygon(
            points={{-96,60},{-96,42},{-96,-38},{-96,-52},{-78,-58},{68,-88},{88,-92},{88,-70},{88,72},{88,94},{68,92},{-76,64},{-96,60}},
            lineThickness=0.5,
            smooth=Smooth.Bezier,
            fillColor={207,211,237},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Line(
            points={{62,88},{62,-84}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{18,80},{18,-76}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{-24,70},{-24,-66}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Line(
            points={{-64,62},{-64,-56}},
            color={157,166,218},
            thickness=0.5,
            smooth=Smooth.Bezier),
          Rectangle(
            extent={{70,4},{-80,0}},
            lineThickness=0.5,
            fillColor={157,166,218},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None)}));
  end TurbineLine_reverse;

  model ParallelTurboFWP_direct
    import MetroscopeModelingLibrary.Units;

    // Initialization parameters
    parameter Units.Pressure STs_CV_P_in_0 = 30e5;
    parameter Units.Pressure STs_P_out_0 = 0.07e5;
    parameter Units.Pressure FWPs_P_in_0 = 44.6e5;
    // BC
    //STs_CV
    input Units.Pressure STs_CV_P_in(start=STs_CV_P_in_0) "Pa";
    input Units.Fraction ST1_CV_opening(start=0.70) "%";
    input Units.Fraction ST2_CV_opening(start=0.70) "%";
    //STs
    input Units.Pressure STs_P_out(start=STs_P_out_0) "condenser pressure, Pa";
    // FWPs
    input Units.Pressure FWPs_P_in(start=FWPs_P_in_0) "Pa";
    input Units.OutletMassFlowRate FWPs_Q_in(start=-1.5e3) "kg/s";
    input Units.Temperature FWPs_T_in(start=186 + 273.15) "degC";

    // Component characteristics
    // STs_CV
    parameter Units.Cv STs_CVmax = 8e3;
    // STs
    parameter Units.Yield ST1_eta_is = 0.8;
    parameter Units.Yield ST2_eta_is = 0.9;
    parameter Units.Cst ST1_Cst = 5e3;
    parameter Units.Cst ST2_Cst = 6e3;
    // FWPs
    parameter Real FWP1_a3 = 634.53723;
    parameter Real FWP2_a3 = 623.4584;
    parameter Real FWPs_b3 = 0.8617299;

    // Components
    // ST sources
    WaterSteam.BoundaryConditions.Source STs_source annotation (Placement(transformation(extent={{-128,70},{-108,90}})));
    WaterSteam.BoundaryConditions.Sink STs_sink annotation (Placement(transformation(extent={{108,70},{128,90}})));
    // STs
    WaterSteam.Machines.StodolaTurbine ST1(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{20,90.0002},{40,110}})));
    WaterSteam.Machines.StodolaTurbine ST2(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{20,70},{40,50}})));
    // STs CV
    WaterSteam.Pipes.ControlValve ST2_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,62.5455},{-4,48.5455}})));
    WaterSteam.Pipes.ControlValve ST1_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,97.4545},{-4,111.455}})));
    Sensors.Other.OpeningSensor ST1_CV_opening_sensor annotation (Placement(transformation(extent={{-14,116},{-6,124}})));
    Sensors.Other.OpeningSensor ST2_CV_opening_sensor annotation (Placement(transformation(extent={{-14,44},{-6,36}})));

    // FWPs
    WaterSteam.BoundaryConditions.Source FWPs_source annotation (Placement(transformation(extent={{128,-70},{108,-50}})));
    WaterSteam.BoundaryConditions.Sink FWPs_sink annotation (Placement(transformation(extent={{-108,-70},{-128,-50}})));

    WaterSteam.Machines.Pump FWP2(P_in_0=FWPs_P_in_0) annotation (Placement(transformation(extent={{-10,-40},{-30,-20}})));
    WaterSteam.Machines.Pump FWP1(P_in_0=FWPs_P_in_0) annotation (Placement(transformation(extent={{-10,-80},{-30,-100}})));
  equation
    // Boundary conditions
    // STs source
    STs_source.h_out = 2.7718e6;
    STs_source.P_out = STs_CV_P_in;

    // ST1 CV
    ST1_CV_opening_sensor.Opening = ST1_CV_opening;

    // ST2_CV
    ST2_CV_opening_sensor.Opening = ST2_CV_opening;

    // STs sink
    STs_sink.P_in = STs_P_out;

    // FWPs
    FWPs_source.T_out = FWPs_T_in;
    FWPs_source.P_out = FWPs_P_in;
    FWPs_source.Q_out = FWPs_Q_in;

    // Component parameters
    // ST1_CV // Hyp on CVs : same Cv (or same mass flow)
    ST1_CV.Cvmax = STs_CVmax;

    // ST1_CV
    ST2_CV.Cvmax = STs_CVmax;

    // ST1
    ST1.eta_nz = 1.0;
    ST1.area_nz = 1.0;
    ST1.eta_is = ST1_eta_is;
    ST1.Cst = ST1_Cst;

    // ST2
    ST2.eta_nz = 1.0;
    ST2.area_nz = 1.0;
    ST2.eta_is = ST2_eta_is;
    ST2.Cst = ST2_Cst;

    // FWP1
    FWP1.VRotn = 4300;
    FWP1.rm = 0.85;
    FWP1.rhmin = 0.20;
    FWP1.b3 = FWPs_b3;
    FWP1.b2 = 0;
    FWP1.b1 = 0;
    FWP1.a3 = FWP1_a3;
    FWP1.a2 = 0;
    FWP1.a1 = -172;

    // FWP2
    FWP2.VRotn = 4500;
    FWP2.rm = 0.85;
    FWP2.rhmin = 0.20;
    FWP2.b3 = FWPs_b3;
    FWP2.b2 = 0;
    FWP2.b1 = 0;
    FWP2.a3 = FWP2_a3;
    FWP2.a2 = 0;
    FWP2.a1 = -172;
    connect(ST1.C_out, ST2.C_out) annotation (Line(points={{40,100},{70,100},{70,60},{40,60}}, color={28,108,200}));
    connect(STs_sink.C_in, ST2.C_out) annotation (Line(points={{113,80},{70,80},{70,60},{40,60}}, color={28,108,200}));
    connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-16,100},{-36,100},{-36,60},{-16,60}}, color={28,108,200}));
    connect(ST1_CV.Opening, ST1_CV_opening_sensor.Opening) annotation (Line(points={{-10,110.182},{-10,115.92}}, color={0,0,127}));
    connect(ST2_CV.Opening, ST2_CV_opening_sensor.Opening) annotation (Line(points={{-10,49.8182},{-10,44.08}}, color={0,0,127}));
    connect(STs_source.C_out, ST2_CV.C_in) annotation (Line(points={{-113,80},{-36,80},{-36,60},{-16,60}}, color={28,108,200}));
    connect(FWP2.C_in, FWP1.C_in) annotation (Line(points={{-10,-30},{60,-30},{60,-90},{-10,-90}},
                                                                                                 color={28,108,200}));
    connect(FWP1.C_out, FWP2.C_out) annotation (Line(points={{-30,-90},{-40,-90},{-40,-30},{-30,-30}},
                                                                                                     color={28,108,200}));
    connect(FWPs_source.C_out, FWP1.C_in) annotation (Line(points={{113,-60},{60,-60},{60,-90},{-10,-90}},color={28,108,200}));
    connect(FWPs_sink.C_in, FWP2.C_out) annotation (Line(points={{-113,-60},{-40,-60},{-40,-30},{-30,-30}},color={28,108,200}));
    connect(ST2.C_W_out, FWP2.C_power) annotation (Line(points={{40,51.6},{62,51.6},{62,-8},{-20,-8},{-20,-19.2}},
                                                                                                                 color={244,125,35}));
    connect(ST1.C_W_out, FWP1.C_power) annotation (Line(points={{40,108.4},{160,108.4},{160,-120},{-20,-120},{-20,-100.8}},
                                                                                                                          color={244,125,35}));
    connect(ST1_CV.C_out, ST1.C_in) annotation (Line(points={{-4,100},{8,100},{8,100},{20,100}}, color={28,108,200}));
    connect(ST2.C_in, ST2_CV.C_out) annotation (Line(points={{20,60},{8,60},{8,60},{-4,60}}, color={28,108,200}));
    annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={127,255,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,0},{80,0}}),
          Line(points={{80,0},{2,60}}),
          Line(points={{80,0},{0,-60}})}));
  end ParallelTurboFWP_direct;

  model ParallelTurboFWP_reverse
    import MetroscopeModelingLibrary.Units;

    // Initialization parameters
    parameter Units.Pressure STs_CV_P_in_0 = 30e5;
    parameter Units.Pressure STs_P_out_0 = 0.07e5;
    parameter Units.Pressure FWPs_P_in_0 = 44.6e5;
    // BC
    //STs_CV
    input Units.Pressure STs_CV_P_in(start=STs_CV_P_in_0) "Pa";
    input Units.Fraction ST1_CV_opening(start=0.70) "%";
    input Units.Fraction ST2_CV_opening(start=0.70) "%";
    //STs
    input Units.Pressure STs_P_out(start=STs_P_out_0) "condenser pressure, Pa";
    // FWPs
    input Units.Pressure FWPs_P_in(start=FWPs_P_in_0) "Pa";
    input Units.OutletMassFlowRate FWPs_Q_in(start=-1.5e3) "kg/s";
    input Units.Temperature FWPs_T_in(start=186 + 273.15) "degC";

    // Observables used for calibration
    // STs_CV
    input Real STs_CV_Q_in(start=13.4) "kg/s";
    // STs
    input Real ST1_P_in(start=10) "barA";
    input Real ST2_P_in(start=10) "barA";
    // FWPs
    input Real FWP1_Q_in(start=783.5) "kg/s";
    input Real FWP1_VRot(start=4253) "rpm";
    input Real FWP2_VRot(start=4206) "rpm";
    input Real FWPs_T_out(start=186.7) "degC";
    input Real FWPs_P_out(start=82) "barA";

    // Component characteristics
    // STs_CV
    output Units.Cv STs_CVmax;
    // STs
    output Units.Yield ST1_eta_is;
    output Units.Yield ST2_eta_is;
    output Units.Cst ST1_Cst;
    output Units.Cst ST2_Cst;
    // FWPs
    output Real FWP1_a3;
    output Real FWP2_a3;
    output Real FWPs_b3;

    // Components
    // ST BCs
    WaterSteam.BoundaryConditions.Source STs_source annotation (Placement(transformation(extent={{-128,70},{-108,90}})));
    WaterSteam.BoundaryConditions.Sink STs_sink annotation (Placement(transformation(extent={{108,70},{128,90}})));
    // STs
    Sensors.WaterSteam.WaterPressureSensor ST1_P_in_sensor annotation (Placement(transformation(extent={{10,93.0001},{24,107}})));
    Sensors.WaterSteam.WaterPressureSensor ST2_P_in_sensor annotation (Placement(transformation(extent={{10,53},{24,67}})));
    WaterSteam.Machines.StodolaTurbine ST1(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{32,90.0002},{52,110}})));
    WaterSteam.Machines.StodolaTurbine ST2(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{32,70},{52,50}})));
    // STs CV
    WaterSteam.Pipes.ControlValve ST2_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,62.5455},{-4,48.5455}})));
    WaterSteam.Pipes.ControlValve ST1_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,97.4545},{-4,111.455}})));
    Sensors.Other.OpeningSensor ST1_CV_opening_sensor annotation (Placement(transformation(extent={{-14,116},{-6,124}})));
    Sensors.Other.OpeningSensor ST2_CV_opening_sensor annotation (Placement(transformation(extent={{-14,44},{-6,36}})));

    // FWP BCs
    WaterSteam.BoundaryConditions.Source FWPs_source annotation (Placement(transformation(extent={{128,-70},{108,-50}})));
    WaterSteam.BoundaryConditions.Sink FWPs_sink annotation (Placement(transformation(extent={{-108,-70},{-128,-50}})));
    // Pumps
    WaterSteam.Machines.Pump FWP2 annotation (Placement(transformation(extent={{-10,-40},{-30,-20}})));
    WaterSteam.Machines.Pump FWP1 annotation (Placement(transformation(extent={{-10,-80},{-30,-100}})));
    Sensors.WaterSteam.WaterFlowSensor STs_CV_Q_in_sensor annotation (Placement(transformation(extent={{-76,73},{-62,87}})));
    Sensors.Other.VRotSensor FWP1_VRot_sensor annotation (Placement(transformation(
          extent={{-7.5,-7.5},{7.5,7.5}},
          rotation=270,
          origin={0,-70})));
    Sensors.Other.VRotSensor FWP2_VRot_sensor annotation (Placement(transformation(
          extent={{-7.5,-7.5},{7.5,7.5}},
          rotation=270,
          origin={0,-50})));
    // Pumps outlet
    Sensors.WaterSteam.WaterTemperatureSensor FWPs_T_out_sensor annotation (Placement(transformation(extent={{-84,-67},{-98,-53}})));
    Sensors.WaterSteam.WaterPressureSensor FWPs_P_out_sensor annotation (Placement(transformation(extent={{-64,-67},{-78,-53}})));
    Sensors.WaterSteam.WaterFlowSensor FWP1_Q_in_sensor annotation (Placement(transformation(extent={{38,-97},{24,-83}})));
  equation
    // Boundary conditions
    // STs source
    STs_source.h_out = 2.7718e6; // set Temp ?
    STs_source.P_out = STs_CV_P_in;

    // ST1 CV
    ST1_CV_opening_sensor.Opening = ST1_CV_opening;

    // ST2_CV
    ST2_CV_opening_sensor.Opening = ST2_CV_opening;

    // STs sink
    STs_sink.P_in = STs_P_out;

    // FWPs
    FWPs_source.T_out = FWPs_T_in;
    FWPs_source.P_out = FWPs_P_in;
    FWPs_source.Q_out = FWPs_Q_in;

    // STs_CV
    // Observables used for calibration
    STs_CV_Q_in_sensor.Q = STs_CV_Q_in;

    // ST1_CV // Hyp on CVs : same Cv (or same mass flow)
    // Calibrated  parameters
    ST1_CV.Cvmax = STs_CVmax;

    // ST1_CV
    // Calibrated parameters
    ST2_CV.Cvmax = STs_CVmax;

    // ST1
    // Observables used for calibration
    ST1_P_in_sensor.P_barA = ST1_P_in;
    // Calibrated parameters
    ST1.eta_nz = 1.0;
    ST1.area_nz = 1.0;
    ST1.eta_is = ST1_eta_is;
    ST1.Cst = ST1_Cst;

    // ST2
    // Observables used for calibration
    ST2_P_in_sensor.P_barA = ST2_P_in;
    // Calibrated parameters
    ST2.eta_nz = 1.0;
    ST2.area_nz = 1.0;
    ST2.eta_is = ST2_eta_is;
    ST2.Cst = ST2_Cst;

    // FWPs
    // Observables used for calibration
    FWPs_T_out_sensor.T_degC = FWPs_T_out;
    FWPs_P_out_sensor.P_barA = FWPs_P_out;

    // FWP1
    // Observables used for calibration
    FWP1_Q_in_sensor.Q = FWP1_Q_in;
    FWP1_VRot_sensor.VRot = FWP1_VRot;
    // Calibrated parameters
    FWP1.b3 = FWPs_b3;
    FWP1.a3 = FWP1_a3;
    // fixed parameters
    FWP1.VRotn = 4300;
    FWP1.rm = 0.85;
    FWP1.rhmin = 0.20;
    FWP1.b2 = 0;
    FWP1.b1 = 0;
    FWP1.a2 = 0;
    FWP1.a1 = -172;

    // FWP2
    // Observables used for calibration
    FWP2_VRot_sensor.VRot = FWP2_VRot;
    // Calibrated parameters
    FWP2.b3 = FWPs_b3;
    FWP2.a3 = FWP2_a3;
    // fixed parameters
    FWP2.VRotn = 4500;
    FWP2.rm = 0.85;
    FWP2.rhmin = 0.20;
    FWP2.b2 = 0;
    FWP2.b1 = 0;
    FWP2.a2 = 0;
    FWP2.a1 = -172;
    connect(ST1.C_out, ST2.C_out) annotation (Line(points={{52,100},{70,100},{70,60},{52,60}}, color={28,108,200}));
    connect(STs_sink.C_in, ST2.C_out) annotation (Line(points={{113,80},{70,80},{70,60},{52,60}}, color={28,108,200}));
    connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-16,100},{-36,100},{-36,60},{-16,60}}, color={28,108,200}));
    connect(ST1_CV.Opening, ST1_CV_opening_sensor.Opening) annotation (Line(points={{-10,110.182},{-10,115.92}}, color={0,0,127}));
    connect(ST2_CV.Opening, ST2_CV_opening_sensor.Opening) annotation (Line(points={{-10,49.8182},{-10,44.08}}, color={0,0,127}));
    connect(FWP1.C_out, FWP2.C_out) annotation (Line(points={{-30,-90},{-40,-90},{-40,-30},{-30,-30}},
                                                                                                     color={28,108,200}));
    connect(ST2.C_W_out, FWP2.C_power) annotation (Line(points={{52,51.6},{62,51.6},{62,-8},{-20,-8},{-20,-19.2}},
                                                                                                                 color={244,125,35}));
    connect(ST1.C_W_out, FWP1.C_power) annotation (Line(points={{52,108.4},{160,108.4},{160,-120},{-20,-120},{-20,-100.8}},
                                                                                                                          color={244,125,35}));
    connect(STs_source.C_out, STs_CV_Q_in_sensor.C_in) annotation (Line(points={{-113,80},{-76,80}}, color={28,108,200}));
    connect(STs_CV_Q_in_sensor.C_out, ST2_CV.C_in) annotation (Line(points={{-62,80},{-36,80},{-36,60},{-16,60}}, color={28,108,200}));
    connect(FWP1.VRot, FWP1_VRot_sensor.VRot) annotation (Line(points={{-20,-78},{-20,-70},{-7.65,-70}}, color={0,0,127}));
    connect(FWP2.VRot, FWP2_VRot_sensor.VRot) annotation (Line(points={{-20,-42},{-20,-50},{-7.65,-50}}, color={0,0,127}));
    connect(FWPs_P_out_sensor.C_in, FWP2.C_out) annotation (Line(points={{-64,-60},{-40,-60},{-40,-30},{-30,-30}}, color={28,108,200}));
    connect(FWPs_sink.C_in, FWPs_T_out_sensor.C_out) annotation (Line(points={{-113,-60},{-98,-60}}, color={28,108,200}));
    connect(FWPs_T_out_sensor.C_in, FWPs_P_out_sensor.C_out) annotation (Line(points={{-84,-60},{-78,-60}}, color={28,108,200}));
    connect(ST1_CV.C_out, ST1_P_in_sensor.C_in) annotation (Line(points={{-4,100},{0,100},{0,100},{4,100},{4,100},{10,100}},
                                                                                             color={28,108,200}));
    connect(ST1_P_in_sensor.C_out, ST1.C_in) annotation (Line(points={{24,100},{26,100},{26,100},{28,100},{28,100},{32,100}},
                                                                                          color={28,108,200}));
    connect(ST2.C_in, ST2_P_in_sensor.C_out) annotation (Line(points={{32,60},{24,60}}, color={28,108,200}));
    connect(ST2_P_in_sensor.C_in, ST2_CV.C_out) annotation (Line(points={{10,60},{4,60},{4,60},{-4,60}}, color={28,108,200}));
    connect(FWPs_source.C_out, FWP1_Q_in_sensor.C_in) annotation (Line(points={{113,-60},{60,-60},{60,-90},{38,-90}}, color={28,108,200}));
    connect(FWP1_Q_in_sensor.C_out, FWP1.C_in) annotation (Line(points={{24,-90},{-10,-90}}, color={28,108,200}));
    connect(FWP2.C_in, FWP1_Q_in_sensor.C_in) annotation (Line(points={{-10,-30},{60,-30},{60,-90},{38,-90}}, color={28,108,200}));
    annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={127,255,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,0},{80,0}}),
          Line(points={{80,0},{2,60}}),
          Line(points={{80,0},{0,-60}})}));
  end ParallelTurboFWP_reverse;

  model FlashTank_Reheater
    import MetroscopeModelingLibrary.Units;
    parameter Units.MassFlowRate Q_cold_0 = 1000;
    parameter Units.MassFlowRate Q_hot_0 = 500;

    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dry_reheater(Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0) annotation (Placement(transformation(extent={{66,22},{34,38}})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink feed_water_sink annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-120,30})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feed_water_source annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={100,30})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_extraction_source
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={50,78})));
    MetroscopeModelingLibrary.WaterSteam.Volumes.FlashTank flashTank
      annotation (Placement(transformation(extent={{-78,-44},{-36,-2}})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source drains_cooling_source
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-78,78})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe reheater_to_flash_tank_DP(Q_0=Q_hot_0)
      annotation (Placement(transformation(
          extent={{-12.5,-12.5},{12.5,12.5}},
          rotation=180,
          origin={0,10})));
    MetroscopeModelingLibrary.WaterSteam.Machines.Pump feed_water_pump(Q_0=Q_hot_0/3) annotation (Placement(transformation(
          extent={{9,-9},{-9,9}},
          rotation=0,
          origin={-60,-80})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe flash_tank_to_reheater_DP(Q_0=Q_hot_0/2) annotation (Placement(transformation(extent={{100,42},{73,69}})));
    MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-60,-52})));
    Sensors.Other.VRotSensor feed_water_pump_VRot_sensor annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=180,
          origin={-60,-110})));
  equation
    // Boundary conditions
    turbine_extraction_source.h_out = 2.5e6;
    turbine_extraction_source.P_out = 11e5;

    feed_water_source.P_out = 50e5;
    feed_water_source.T_out = 130 + 273.15;
    feed_water_source.Q_out = -1000;

    drains_cooling_source.h_out = 8e5;
    drains_cooling_source.Q_out = -Q_hot_0;

    // Reheater
    dry_reheater.S_condensing = 100;
    dry_reheater.Kfr_hot = 1;
    dry_reheater.Kfr_cold = 1;
    dry_reheater.Kth = 1e5;

    // Pressure losses
    reheater_to_flash_tank_DP.delta_z = -10;
    reheater_to_flash_tank_DP.Kfr = 1;
    flash_tank_to_reheater_DP.delta_z = 0;

    // Pump
    // Observables used for calibration
    feed_water_pump.VRot = 1000;
    // Fixed parameters
    feed_water_pump.VRotn = 1000;
    feed_water_pump.rm = 1;
    feed_water_pump.a1 = 0;
    feed_water_pump.a2 = 0;
    feed_water_pump.b1 = 0;
    feed_water_pump.b2 = 0;
    feed_water_pump.rhmin = 0.20;
    feed_water_pump.rh = 1;

    connect(power_source.C_out, feed_water_pump.C_power) annotation (Line(points={{-60,-56.8},{-60,-70.28}}, color={244,125,35}));
    connect(turbine_extraction_source.C_out, dry_reheater.C_hot_in) annotation (Line(points={{50,73},{50,38}}, color={63,81,181}));
    connect(feed_water_source.C_out, dry_reheater.C_cold_in) annotation (Line(points={{95,30},{66.2,30}}, color={63,81,181}));
    connect(feed_water_sink.C_in, dry_reheater.C_cold_out) annotation (Line(points={{-115,30},{34,30}}, color={63,81,181}));
    connect(drains_cooling_source.C_out, flashTank.C_in)
      annotation (Line(points={{-78,73},{-78,-14},{-78,-14},{-78,-14.6}},
                                                        color={63,81,181}));
    connect(dry_reheater.C_hot_out, reheater_to_flash_tank_DP.C_in) annotation (Line(points={{50,22},{50,10},{12.5,10}}, color={63,81,181}));
    connect(reheater_to_flash_tank_DP.C_out, flashTank.C_in) annotation (Line(points={{-12.5,10},{-78,10},{-78,-14.6}}, color={63,81,181}));
    connect(feed_water_pump.C_out, feed_water_sink.C_in) annotation (Line(points={{-69,-80},{-99,-80},{-99,30},{-115,30}}, color={63,81,181}));
    connect(flash_tank_to_reheater_DP.C_out, dry_reheater.C_hot_in) annotation (Line(points={{73,55.5},{50,55.5},{50,38}}, color={63,81,181}));
    connect(flashTank.C_hot_liquid, feed_water_pump.C_in) annotation (Line(points={{-36,-31.4},{-36,-30},{-28,-30},{-28,-80},{-51,-80}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, flash_tank_to_reheater_DP.C_in) annotation (Line(points={{-36,-14.6},{112,-14.6},{112,55.5},{100,55.5}}, color={28,108,200}));
    connect(feed_water_pump.VRot, feed_water_pump_VRot_sensor.VRot) annotation (Line(points={{-60,-90.8},{-60,-101.84}}, color={0,0,127}));
    annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
          Rectangle(
            extent={{-94,62},{106,-60}},
            lineColor={28,108,200},
            fillColor={236,238,248},
            fillPattern=FillPattern.Solid,
            lineThickness=1),
          Polygon(
            points={{-94,-30},{106,-30},{106,-60},{-94,-60},{-94,-30}},
            lineColor={28,108,200},
            lineThickness=0.5,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-82,2},{-76,-4}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-74,-10},{-68,-16}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-68,6},{-62,0}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-86,-20},{-80,-26}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-78,14},{-72,8}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-62,-14},{-56,-20}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-86,40},{-80,34}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-62,18},{-56,12}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-48,-22},{-42,-28}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-88,-6},{-82,-12}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-86,22},{-80,16}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-70,-20},{-64,-26}},
            lineThickness=1,
            fillColor={79,188,247},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-94,62},{106,-60}},
            lineColor={28,108,200},
            lineThickness=1)}));
  end FlashTank_Reheater;
end Nuclear;
