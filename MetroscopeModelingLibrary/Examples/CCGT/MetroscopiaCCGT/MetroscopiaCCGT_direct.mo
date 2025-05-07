within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_direct
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
    // Circulating water circuit
    input Real P_circulating_water_in(start=5, min=0, nominal=5) "barA";
    input Real T_circulating_water_in(start = 15, min = 0, nominal = 15) "degC";
    // Flue gas sink
    input Real P_flue_gas_sink(start=1, min=0, nominal=1) "barA";
    input Units.SpecificEnthalpy LHV_plant(start=48130e3) "Directly assigned in combustion chamber modifiers";

  // Parameters

    // Gas Turbine
    parameter Real turbine_T_out = 640 "degC";
    parameter Real combustionChamber_eta = 0.9999;
    parameter Units.FrictionCoefficient combustionChamber_Kfr = 1e-3;
    // Economizer
    parameter String Eco_QCp_max_side = "unidentified";
    parameter Real T_w_eco_in = 85 "degC"; // Controlled by the economizer recirculation pump flow rate
    // Evaporator
    parameter Real Evap_x_steam_out=1;
    // High Pressure Superheater
    parameter String HPSH_QCp_max_side = "hot";
    // High Pressure Superheater 2
    parameter Real T_w_HPSH2_out = 566.5 "degC"; // Controlled by the desuperheater mass flow rate
    // Low Pressure Superheater (resuperheater)
    parameter String ReH_QCp_max_side = "hot";

  // Observables

    // Gas Turbine
    output Real P_filter_out;
    output Real compressor_P_out;
    output Real compressor_T_out;
    output Real W_GT;
    output Real turbine_P_out;
    output Units.MassFlowRate Q_fuel_source; // Controlled by the gas turbine outlet temperature 10.5
    output Real turbine_compression_rate;
    // Economizer
    output Real P_w_eco_out;
    output Real T_w_eco_out;
    output Real T_flue_gas_sink;
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
    // Steam Turbine
    output Real P_ST_in;
    output Real P_ST_out;
    output Real T_HPST_out "degC";
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
    output Real Q_pumpRec_out; // Controlled by the economizer input temperature

  // Calibrated parameters (input used for calibration in comment)

    // Gas Turbine
    parameter Units.FrictionCoefficient Filter_Kfr=0.04432005; // Filter outlet pressure
    parameter Real compression_rate = 18.88889; // Air compressor outlet pressure
    parameter Real compressor_eta_is = 0.878675; // Air compressor outlet temperature
    parameter Real turbine_eta_is = 0.8304104; // Gas turbine power output
    // Economizer
    parameter Units.HeatExchangeCoefficient Eco_Kth=3104.9373; // Economizer water outlet temperature
    parameter Units.FrictionCoefficient Eco_Kfr_cold=973146.4; // Economizer water outlet pressure
    // Evaporator
    parameter Units.Cv Evap_CV_Cvmax=539.1173; // Evaporator control valve opening
    parameter Units.HeatExchangeCoefficient Evap_Kth=3383.7917; // Extraction pump mass flow rate
    // High Pressure Superheater 1
    parameter Units.HeatExchangeCoefficient HPSH1_Kth=1213.6362; // HP superheater outlet temperature
    parameter Units.FrictionCoefficient HPSH1_Kfr_cold=7030.31; // HP superheater inlet pressure
    // High Pressure Superheater 2
    parameter Units.HeatExchangeCoefficient HPSH2_Kth=1673.8336; // De-superheater mass flow rate
    parameter Units.FrictionCoefficient HPSH2_Kfr_cold=2538.3271; // HP superheater inlet pressure
    // De-superheater
    parameter Units.Cv deSH_CV_Cvmax=7.7502966; // Desuperheater control valve opening
    // Reheater
    parameter Units.HeatExchangeCoefficient ReH_Kth=410.44293; // LP superheater outlet temperature
    parameter Units.FrictionCoefficient ReH_Kfr_cold=134.2858; // LP superheater inlet pressure
    // High Pressure Steam Turbine
    parameter Units.Cv HPST_CV_Cv=6647.2905; // HP superheater outlet pressure
    parameter Units.Cst HPST_Cst=6.038082e+07; // HP steam turbine inlet pressure
    parameter Units.Yield HPST_eta_is=0.8438316; // HP steam turbine outlet temperature
    parameter Units.Yield LPST_eta_is=0.8438316; // Power output
    // Low Pressure Steam Turbine
    parameter Units.Cv LPST_CV_Cv=69310.586; // Low pressure superheater outlet pressure
    parameter Units.Cst LPST_Cst=411424.22; // LP steam turbine inlet pressure
    // Condenser
    parameter Units.HeatExchangeCoefficient Cond_Kth=93661.23; // Condensation pressure
    parameter Units.VolumeFlowRate Qv_cond_cold=2.7349906; // Circulating water outlet temperature
    // Exctraction Pump
    parameter Real pump_hn = 1735.4259; // Exctraction pump outlet pressure
    parameter Real pump_rh = 0.70563865; // Exctraction pump outlet temperature
    // Recirculation pump
    parameter Real pumpRec_hn = 870.66187; // Recirculation pump outlet pressure
    parameter Real pumpRec_rh = 0.6881236; // Recirculation pump outlet temperature
    parameter Real pumpRec_CV_Cvmax = 52.329174; // Recirculation control valve opening

  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Economiser economiser(
    nominal_DT_default=false,
      QCp_max_side=Eco_QCp_max_side,
    Q_cold_0=56.89394,
    Q_hot_0=510.45065,
    T_cold_in_0=358.14999,
    T_cold_out_0=593.15,
    T_hot_in_0=719.15,
    T_hot_out_0=612.15,
    P_cold_in_0=17000533,
    P_cold_out_0=13770425,
    P_hot_out_0=100000,
    h_cold_in_0=369317.84,
    h_cold_out_0=1459478.5,
    h_hot_in_0=772388.4,
    h_hot_out_0=650881)
    annotation (Placement(transformation(extent={{84,-20.5},{124,20}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink flue_gas_sink
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={222,224})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_eco_out_sensor(
    Q_0=47.517586,
    P_0=11720722,
    h_0=1459478.5,
    T_0=592.94025)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={8,34})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_eco_out_sensor(
    Q_0=47.517586,
    P_0=13770425,
    h_0=1459478.5)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={56,34})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Evaporator evaporator(
    x_steam_out(start=1),
    faulty=false,
    Q_cold_0=47.517586,
    Q_hot_0=510.45065,
    T_cold_in_0=592.94025,
    P_cold_in_0=11720722,
    P_cold_out_0=11720722,
    P_hot_out_0=110000)
    annotation (Placement(transformation(extent={{-36,-22},{6,20}})));

  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_evap_out_sensor(
    Q_0=47.517586,
    P_0=11720722,
    h_0=2691576)
    annotation (Placement(transformation(extent={{-34,28},{-46,40}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater1(
    nominal_DT_default=false,
      QCp_max_side="unidentified",
    Q_cold_0=47.517586,
    Q_hot_0=510.45065,
    T_cold_in_0=596.03474,
    T_cold_out_0=726.45,
    T_hot_in_0=878.034,
    T_hot_out_0=835.819,
    P_cold_in_0=11720000,
    P_cold_out_0=11484281,
    P_hot_in_0=111000,
    P_hot_out_0=111000,
    h_cold_in_0=2691576,
    h_cold_out_0=3227785.8,
    h_hot_in_0=957089.1,
    h_hot_out_0=907173.6)
    annotation (Placement(transformation(extent={{-186,-22},{-144,20}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_HPSH1_out_sensor(Q_0=47.517586,
    P_0=11484281,
    h_0=3227785.8)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-184,34})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_HPSH1_out_sensor(
    Q_0=47.517586,
    P_0=11484281,
    h_0=3227785.8)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-208,34})));
  WaterSteam.Pipes.SlideValve                             HPST_control_valve(
    T_in_0=839.65,
    T_out_0=839.2726,
    P_in_0=11338658,
    P_out_0=11238561,
    h_in_0=3530374.2,
    h_out_0=3530374.2,
    Q_0=49.734425,
    T_0=839.65,
    h_0=3530374.2)
    annotation (Placement(transformation(extent={{-203.25,176.738},{-186.75,194.677}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_in_sensor(
    Q_0=49.734425,
    P_0=11238561,
    h_0=3530374.2)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-174,180})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPsteamTurbine(
    T_in_0=839.2726,
    T_out_0=527.15528,
    P_in_0=11238561,
    P_out_0=978559.2,
    h_in_0=3530374.2,
    h_out_0=2952838,
    Q_0=49.734425,
    x_out_0=1,
    xm_0=1,
    x_in_0=1)                                                               annotation (Placement(transformation(extent={{-160,164},{-126,196}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_HPST_out_sensor(
    Q_0=49.734425,
    P_0=978559.2,
    h_0=2952838)
    annotation (Placement(transformation(extent={{-114,174},{-102,186}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_ST_out_sensor(W_MW(start=64.77))
    annotation (Placement(transformation(extent={{90,274},{102,286}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser(
    Q_cold_0=2730.2332,
    Q_hot_0=49.734425,
    Psat_0=4973.1817,
    P_cold_in_0=500000,
    P_cold_out_0=492540.5,
    T_cold_in_0=288.151703,
    T_cold_out_0=298.096405,
    T_hot_out_0=305.92236,
    h_cold_in_0=63375,
    h_cold_out_0=104974.11,
    h_hot_in_0=2420969,
    water_height_DP_0=10000)
    annotation (Placement(transformation(extent={{32,170.778},{72,202.778}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_circulating_water_out_sensor(
    Q_0=2730.2332,
    P_0=492540.5,
    h_0=104974.11,
    T_0=298.096405)
    annotation (Placement(transformation(extent={{86,197},{96,207}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source circulating_water_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-16,185})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink circulating_water_sink
    annotation (Placement(transformation(extent={{98,194},{114,210}})));
  WaterSteam.Machines.FixedSpeedPump                 pump(
    T_in_0=305.940398,
    T_out_0=308.05325,
    P_in_0=14728.373,
    P_out_0=17000533,
    h_in_0=137334.1,
    h_out_0=161452.27,
    rho_0=998,
    Q_0=49.734425)                                        annotation (Placement(
        transformation(
        extent={{-7,-7},{7,7}},
        origin={110,157},
        rotation=0)));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_pump_out_sensor(
    Q_0=49.734425,
    P_0=17000533,
    h_0=161452.27,
    mass_flow_rate_bias(start=34.90325))
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={137,157})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_pump_out_sensor(
    Q_0=49.734425,
    P_0=17000533,
    h_0=161452.27)
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={155,157})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={180,54})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_pump_out_sensor(
    Q_0=49.734425,
    P_0=17000533,
    h_0=161452.27,
    Qv_0=0.04966522)
    annotation (Placement(transformation(extent={{166,152},{176,162}})));
  MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor(
    T_in_0=297.149994,
    T_out_0=723.15003,
    P_in_0=89940.85,
    P_out_0=1698882.9,
    h_in_0=301935.4,
    h_out_0=749620.4,
    Q_0=500)
    annotation (Placement(transformation(extent={{-524,-14},{-496,14}})));
  MetroscopeModelingLibrary.FlueGases.Machines.GasTurbine gasTurbine(
    T_in_0=1500.8623,
    T_out_0=913.15,
    P_in_0=1698815.2,
    P_out_0=111669.16,
    h_in_0=1737970.2,
    h_out_0=998985.06,
    Q_0=510.45065,
    eta_is(start=0.73),
    eta_mech(start=0.9))
    annotation (Placement(transformation(extent={{-412,-16},{-382,15}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_power
    annotation (Placement(transformation(extent={{-332,50},{-312,70}})));
  MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber(LHV=LHV_plant, h_in_air_0=749620.4)
    annotation (Placement(transformation(extent={{-452,-10},{-432,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source_fuel(h_out(
        start=0.9e6)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-442,-64})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor compressor_P_out_sensor(
    Q_0=500,
    P_0=1698882.9,
    h_0=749620.4)
    annotation (Placement(transformation(extent={{-490,-6},{-478,6}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor(
    Q_0=500,
    P_0=100000,
    h_0=749620.4,
    T_0=723.15)
    annotation (Placement(transformation(extent={{-472,-6},{-460,6}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor turbine_P_out_sensor(
    Q_0=510.45065,
    P_0=111669.16,
    h_0=998985.06)
    annotation (Placement(transformation(extent={{-350,-7},{-338,5}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_GT_sensor
    annotation (Placement(transformation(extent={{-346,54},{-334,66}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor turbine_T_out_sensor(
    Q_0=510.45065,
    P_0=111669.16,
    h_0=998985.06,
    sensor_function="BC",
    T_0=913.15)
    annotation (Placement(transformation(extent={{-370,-7},{-358,5}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater Reheater(
    nominal_DT_default=false,
      QCp_max_side=ReH_QCp_max_side,
    Q_cold_0=49.734425,
    Q_hot_0=510.45065,
    T_cold_in_0=527.15528,
    T_cold_out_0=622.15,
    T_hot_in_0=835.85,
    T_hot_out_0=818.75,
    P_cold_in_0=978559.2,
    P_cold_out_0=895239.5,
    P_hot_in_0=111000,
    P_hot_out_0=111000,
    h_cold_in_0=2952838,
    h_cold_out_0=3159031.2,
    h_hot_in_0=907173.6,
    h_hot_out_0=887083.75)
    annotation (Placement(transformation(extent={{-92,-22},{-50,20}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPsteamTurbine(
    T_in_0=621.62244,
    T_out_0=305.92236,
    P_in_0=795378.5,
    P_out_0=4973.1817,
    h_in_0=3159031.2,
    h_out_0=2420969,
    Q_0=49.734425,
    x_out_0=0.9423811,
    xm_0=0.9711905,
    x_in_0=1)                                                               annotation (Placement(transformation(extent={{-14,224},{20,256}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_ReH_out_sensor(
    Q_0=49.734425,
    P_0=895239.5,
    h_0=3159031.2,
    T_0=622.56934)
    annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=270,
        origin={-80,55})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_ReH_out_sensor(
    Q_0=49.734425,
    P_0=895239.5,
    h_0=3159031.2)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-80,75})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_Cond_sensor(
    Q_0=49.734425,
    P_0=4973.1817,
    h_0=2420969)
    annotation (Placement(transformation(extent={{28,234},{40,246}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_source_air_sensor(
    Q_0=500,
    P_0=100000,
    h_0=301935.4,
    sensor_function="BC")
    annotation (Placement(transformation(extent={{-636,-6},{-624,6}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_source_air_sensor(
    Q_0=500,
    P_0=100000,
    h_0=301935.4,
    sensor_function="BC")
    annotation (Placement(transformation(extent={{-618,-6},{-606,6}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlowSensor Q_source_air_sensor(
    Q_0=500,
    P_0=100000,
    h_0=301935.4,
    sensor_function="BC")
    annotation (Placement(transformation(extent={{-600,-6},{-588,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_circulating_water_in_sensor(
    Q_0=2730.2332,
    P_0=500000,
    h_0=63375,
    sensor_function="BC",
    T_0=288.149994)
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={3,185})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_circulating_water_in_sensor(
    Q_0=2730.2332,
    P_0=500000,
    h_0=63375,
    sensor_function="BC")
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={19,185})));
  WaterSteam.Pipes.SlideValve                             LPST_control_valve(
    T_in_0=622.56934,
    T_out_0=621.62244,
    P_in_0=895239.5,
    P_out_0=795375.8,
    h_in_0=3159031.2,
    h_out_0=3159031.2,
    Q_0=49.734425,
    T_0=622.56934,
    h_0=3159031.2)
    annotation (Placement(transformation(extent={{-61.25,236.738},{-44.75,254.677}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_LPST_in_sensor(
    Q_0=49.734425,
    P_0=795375.8,
    h_0=3530374.2)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={-28,240})));
  WaterSteam.Machines.FixedSpeedPump                 pumpRec(
    T_in_0=593.66584,
    T_out_0=597.50715,
    P_in_0=13770425,
    P_out_0=19550464,
    h_in_0=1459478.5,
    h_out_0=1471886.6,
    Q_0=9.376354)
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        origin={94,80.5455},
        rotation=0)));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_pumpRec_out_sensor(
    Q_0=9.376354,
    P_0=19550464,
    h_0=1471886.6,
    T_0=597.50715)
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={115,80.5455})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_pumpRec_out_sensor(
    Q_0=9.376354,
    P_0=19550464,
    h_0=1471886.6)
    annotation (Placement(transformation(extent={{-5,-5},{5,5}}, origin={131,80.5455})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_pumpRec_out_sensor(
    Q_0=9.376354,
    P_0=19550464,
    h_0=1471886.6,
    Qv_0=0.013783708)
    annotation (Placement(transformation(extent={{140,75.5455},{150,85.5455}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_eco_in_sensor(
    Q_0=56.89394,
    P_0=17000533,
    h_0=369317.84,
    sensor_function="BC",
    T_0=358.14999)
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=180,
        origin={145,35})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve pumpRec_controlValve(
    T_in_0=597.50715,
    T_out_0=596.7196,
    P_in_0=19550464,
    P_out_0=17000533,
    h_in_0=1471886.6,
    h_out_0=1471886.6,
    Q_0=9.376354,
    T_0=597.50715,
    h_0=1471886.6)
    annotation (Placement(transformation(extent={{160,78},{172,92}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor pumpRec_opening_sensor(Opening_pc_0=21.797726)
    annotation (Placement(transformation(extent={{162,100},{172,110}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_flue_gas_sink_sensor(Q_0=510.45065, h_0=650881,
    sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={222,198})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{110,270},{130,290}})));
  MetroscopeModelingLibrary.Sensors.Fuel.PressureSensor P_fuel_source_sensor(
    Q_0=10.450661,
    P_0=3000000,
    h_0=899265,
    sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-35})));
  MetroscopeModelingLibrary.Sensors.Fuel.TemperatureSensor T_fuel_source_sensor(
    Q_0=10.450661,
    P_0=3000000,
    h_0=899265,
    sensor_function="BC")
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-49})));
  MetroscopeModelingLibrary.Sensors.Fuel.FlowSensor Q_fuel_source_sensor(
    Q_0=10.450661,
    P_0=3000000,
    h_0=899265)
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=90,
        origin={-442,-21})));
  MetroscopeModelingLibrary.Power.Machines.Generator GT_generator
    annotation (Placement(transformation(extent={{-380,50},{-348,70}})));
  MetroscopeModelingLibrary.Power.Machines.Generator ST_generator
    annotation (Placement(transformation(extent={{50,270},{82,290}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_flue_gas_sink_sensor(
    Q_0=510.45065,
    P_0=100000,
    h_0=650881,
    T_0=612.48182)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={176,-1})));
  MetroscopeModelingLibrary.FlueGases.Pipes.Filter AirFilter(
    T_in_0=297.149994,
    T_out_0=297.149994,
    P_in_0=100000,
    P_out_0=89940.85,
    h_in_0=301935.4,
    h_out_0=301935.4,
    Q_0=500,
    T_0=297.149994,
    h_0=301935.4)
    annotation (Placement(transformation(extent={{-576,-10},{-556,10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_filter_out_sensor(
    Q_0=500,
    P_0=100000,
    h_0=301935.4)
    annotation (Placement(transformation(extent={{-548,-6},{-536,6}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater HPsuperheater2(
    nominal_DT_default=false,
      QCp_max_side=HPSH_QCp_max_side,
    Q_cold_0=49.734425,
    Q_hot_0=510.45065,
    T_cold_in_0=684.24146,
    T_cold_out_0=839.65,
    T_hot_in_0=913.15,
    T_hot_out_0=878.034,
    P_cold_in_0=11484000,
    P_cold_out_0=11338000,
    P_hot_in_0=111000,
    P_hot_out_0=111000,
    h_cold_in_0=3100373.8,
    h_cold_out_0=3530374.2,
    h_hot_in_0=998985.06,
    h_hot_out_0=957089.1)
    annotation (Placement(transformation(extent={{-292,-20},{-254,18}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_w_HPSH2_out_sensor(
    Q_0=49.734425,
    P_0=11338658,
    h_0=3530374.2,
    sensor_function="BC",
    T_0=839.65)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-280,60})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_w_HPSH2_out_sensor(
    Q_0=49.734425,
    P_0=11338658,
    h_0=3530374.2)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-280,78})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve deSH_controlValve(
    P_in_0=17000533,
    P_out_0=11484281,
    Q_0=2.2168374,
    T_0=358.14999,
    h_0=369317.84)
    annotation (Placement(transformation(extent={{-158.75,117.454},{-171.25,131.455}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor deSH_opening_sensor(Opening_pc_0=16.4507)
    annotation (Placement(transformation(extent={{-170,142},{-160,152}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_deSH_sensor(
    Q_0=2.2168374,
    P_0=17000533,
    h_0=369317.84,
    Qv_0=0.002271126)
    annotation (Placement(transformation(extent={{-132,114},{-144,126}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve Evap_controlValve(
    P_in_0=13770425,
    P_out_0=11720722,
    Q_0=47.517586,
    T_0=593.66584,
    h_0=1459478.5)
    annotation (Placement(transformation(extent={{41.25,31.4545},{28.75,45.455}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor Evap_opening_sensor(Opening_pc_0=12.070684)
    annotation (Placement(transformation(extent={{30,60},{40,70}})));
  MoistAir.BoundaryConditions.Source source_air(relative_humidity_0=0.5, h_out(start=47645.766), Xi_out(start={0.01}))
                                                annotation (Placement(transformation(extent={{-718,-10},{-698,10}})));
  MultiFluid.Converters.MoistAir_to_FlueGases moistAir_to_FlueGases annotation (Placement(transformation(extent={{-672,-10},{-652,10}})));
  Sensors.WaterSteam.TemperatureSensor T_HPST_out_sensor(
    Q_0=49.734425,
    P_0=978559.2,
    h_0=2952838,
    T_0=527.15528)                                       annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=180,
        origin={-90,180})));
  Sensors.MoistAir.RelativeHumiditySensor H_sensor(sensor_function="BC") annotation (Placement(transformation(extent={{-694,-6},{-682,6}})));
  FlueGases.Pipes.FrictionPipe HRSG_friction annotation (Placement(transformation(extent={{40,10},{60,-10}})));
  Utilities.Interfaces.RealInput HRSG_Kfr(start=0.022388678)
                                          annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={50,-20}), iconTransformation(extent={{-36,-16},{4,24}})));
equation

  //--- Air / Flue Gas System ---

    // Air source
      // Quantities definition
      P_source_air_sensor.P_barA = P_source_air;
      T_source_air_sensor.T_degC = T_source_air;
      Q_source_air_sensor.Q = Q_source_air;
      H_sensor.relative_humidity=Relative_Humidity;

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
    HRSG_friction.delta_z = 0;

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
      economiser.Kfr_cold = Eco_Kfr_cold;

    // Evaporator
      // Quantities definition
      P_w_evap_out_sensor.P_barA = P_w_evap_out;
      evaporator.x_steam_out = Evap_x_steam_out;
      Evap_opening_sensor.Opening = Evap_opening;
      // Parameters
      evaporator.S = 100;
      // Calibrated parameters
  Evap_controlValve.Cv_max = Evap_CV_Cvmax;
      evaporator.Kth = Evap_Kth;

    // HP Superheater 1
      // Quantities definition
      P_w_HPSH1_out_sensor.P_barA = P_w_HPSH1_out;
      T_w_HPSH1_out_sensor.T_degC = T_w_HPSH1_out;
      // Parameters
      HPsuperheater1.S = 100;
      HPsuperheater1.nominal_cold_side_temperature_rise = 250;
      HPsuperheater1.nominal_hot_side_temperature_drop = 180;
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
      T_w_HPSH2_out_sensor.T_degC = T_w_HPSH2_out;
      // Calibrated parameters
      HPsuperheater2.Kth = HPSH2_Kth;
      HPsuperheater2.Kfr_cold = HPSH2_Kfr_cold;

    // De-superheater
      // Quantities definition
      deSH_opening_sensor.Opening = deSH_opening;
      Q_deSH_sensor.Q = Q_deSH;
      // Calibrated parameters
  deSH_controlValve.Cv_max = deSH_CV_Cvmax;

    // Reheater
      // Quantities definition
      T_w_ReH_out_sensor.T_degC = T_w_ReH_out;
      P_w_ReH_out_sensor.P_barA = P_w_ReH_out;
      // Parameters
      Reheater.S = 100;
      Reheater.nominal_cold_side_temperature_rise = 100;
      Reheater.nominal_hot_side_temperature_drop = 180;
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
        //HPsteamTurbine.eta_is = LPsteamTurbine.eta_is;
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

      // Calibrated parameters
      pump.hn = pump_hn;
      pump.rh = pump_rh;

    //--- Recirculation pump ---
      // Quantities definition
      T_pumpRec_out_sensor.T_degC = T_pumpRec_out;
      P_pumpRec_out_sensor.P_barA = P_pumpRec_out;
      pumpRec_opening_sensor.Opening = pumpRec_opening;
      Q_pumpRec_out_sensor.Q = Q_pumpRec_out;

      // Calibrated parameters
      pumpRec.hn = pumpRec_hn;
      pumpRec.rh = pumpRec_rh;
  pumpRec_controlValve.Cv_max = pumpRec_CV_Cvmax;

  connect(HPsuperheater1.C_cold_out, T_w_HPSH1_out_sensor.C_in) annotation (
     Line(points={{-173.4,15.8},{-172,15.8},{-172,34},{-178,34}},
                                                          color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_HPST_out_sensor.C_in, HPsteamTurbine.C_out)
    annotation (Line(points={{-114,180},{-126,180}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_HPST_in_sensor.C_out, HPsteamTurbine.C_in)
    annotation (Line(points={{-168,180},{-160,180}}, color={28,108,200},
      pattern=LinePattern.Dash));
  connect(T_circulating_water_out_sensor.C_out, circulating_water_sink.C_in)
    annotation (Line(points={{96,202},{102,202}},color={28,108,200}));
  connect(combustionChamber.outlet,gasTurbine. C_in) annotation (Line(points={{-432,0},{-412,0},{-412,-0.5}},
                                                                                              color={95,95,95},
      thickness=1));
  connect(airCompressor.C_out,compressor_P_out_sensor. C_in) annotation (Line(points={{-496,0},{-490,0}},
                                                                                                        color={95,95,95},
      thickness=1));
  connect(compressor_P_out_sensor.C_out,compressor_T_out_sensor. C_in) annotation (Line(points={{-478,0},{-472,0}},
                                                                                                                  color={95,95,95},
      thickness=1));
  connect(compressor_T_out_sensor.C_out,combustionChamber. inlet) annotation (Line(points={{-460,0},{-452,0}},
                                                                                                             color={95,95,95},
      thickness=1));
  connect(evaporator.C_cold_in, T_w_eco_out_sensor.C_out) annotation (Line(
        points={{-6.6,15.8},{-6,15.8},{-6,34},{2,34}},         color={28,108,200},
      thickness=1));
  connect(condenser.C_cold_out, T_circulating_water_out_sensor.C_in)
    annotation (Line(points={{71.6,185},{74,185},{74,184},{78,184},{78,202},{86,202}},
                    color={28,108,200}));
  connect(condenser.C_hot_out, pump.C_in) annotation (Line(points={{52,170.778},{52,157},{103,157}},
                              color={28,108,200},
      thickness=1));
  connect(HPsuperheater1.C_cold_in, P_w_evap_out_sensor.C_out) annotation (Line(
        points={{-156.6,15.8},{-156,15.8},{-156,34},{-46,34}},
                                              color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(evaporator.C_cold_out, P_w_evap_out_sensor.C_in) annotation (Line(
        points={{-23.4,15.8},{-22,15.8},{-22,34},{-34,34}},
                                                        color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(gasTurbine.C_out, turbine_T_out_sensor.C_in)
    annotation (Line(points={{-382,-0.5},{-382,-1},{-370,-1}},
                                                     color={95,95,95},
      thickness=1));
  connect(turbine_P_out_sensor.C_in, turbine_T_out_sensor.C_out)
    annotation (Line(points={{-350,-1},{-358,-1}},   color={95,95,95},
      thickness=1));
  connect(evaporator.C_hot_in, Reheater.C_hot_out) annotation (Line(points={{-36,-1},{-50,-1}},
                                           color={95,95,95},
      thickness=1));
  connect(HPsuperheater1.C_hot_out, Reheater.C_hot_in)
    annotation (Line(points={{-144,-1},{-92,-1}},   color={95,95,95},
      thickness=1));
  connect(Reheater.C_cold_out, T_w_ReH_out_sensor.C_in)
    annotation (Line(points={{-79.4,15.8},{-79.4,30},{-80,30},{-80,49}},
                                                          color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_Cond_sensor.C_in, LPsteamTurbine.C_out)
    annotation (Line(points={{28,240},{20,240}},   color={28,108,200},
      thickness=1));
  connect(P_Cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{40,240},{52,240},{52,203.134}},
                                     color={28,108,200},
      thickness=1));

  connect(P_source_air_sensor.C_out, T_source_air_sensor.C_in)
    annotation (Line(points={{-624,0},{-618,0}},     color={95,95,95},
      thickness=1));
  connect(T_source_air_sensor.C_out, Q_source_air_sensor.C_in)
    annotation (Line(points={{-606,0},{-600,0}},     color={95,95,95},
      thickness=1));
  connect(condenser.C_cold_in, P_circulating_water_in_sensor.C_out) annotation (
     Line(points={{32,185},{28,185},{28,185},{24,185}},               color={28,
          108,200}));
  connect(P_LPST_in_sensor.C_out, LPsteamTurbine.C_in)
    annotation (Line(points={{-22,240},{-14,240}},  color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_LPST_in_sensor.C_in, LPST_control_valve.C_out) annotation (Line(
        points={{-34,240},{-40,240},{-40,240},{-44.75,240}},             color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_ReH_out_sensor.C_out, LPST_control_valve.C_in) annotation (Line(
        points={{-80,81},{-80,240},{-61.25,240}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_ReH_out_sensor.C_in, T_w_ReH_out_sensor.C_out)
    annotation (Line(points={{-80,69},{-80,61}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(T_w_HPSH1_out_sensor.C_out, P_w_HPSH1_out_sensor.C_in)
    annotation (Line(points={{-190,34},{-202,34}},
                                                 color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(HPST_control_valve.C_out, P_HPST_in_sensor.C_in) annotation (Line(
        points={{-186.75,180},{-184,180},{-184,180},{-180,180}}, color={28,108,200},
      pattern=LinePattern.Dash));
  connect(T_circulating_water_in_sensor.C_out, P_circulating_water_in_sensor.C_in)
    annotation (Line(points={{8,185},{14,185}},    color={28,108,200}));
  connect(circulating_water_source.C_out, T_circulating_water_in_sensor.C_in)
    annotation (Line(points={{-11,185},{-2,185}},            color={28,108,200}));
  connect(pump.C_out, T_pump_out_sensor.C_in)
    annotation (Line(points={{117,157},{132,157}},     color={28,108,200},
      thickness=1));
  connect(T_pump_out_sensor.C_out, P_pump_out_sensor.C_in) annotation (Line(
        points={{142,157},{150,157}},             color={28,108,200},
      thickness=1));
  connect(P_pump_out_sensor.C_out, Q_pump_out_sensor.C_in) annotation (Line(
        points={{160,157},{166,157}},             color={28,108,200},
      thickness=1));
  connect(P_pumpRec_out_sensor.C_out, Q_pumpRec_out_sensor.C_in)
    annotation (Line(points={{136,80.5455},{140,80.5455}},
                                               color={28,108,200},
      thickness=0.5));
  connect(pumpRec_controlValve.Opening, pumpRec_opening_sensor.Opening)
    annotation (Line(points={{166,90.7273},{166,94},{167,94},{167,99.9}},
                                                                color={0,0,127}));
  connect(pumpRec.C_in, P_w_eco_out_sensor.C_in) annotation (Line(points={{87,80.5455},{74,80.5455},{74,34},{62,34}},
                                   color={28,108,200},
      thickness=0.5));
  connect(economiser.C_cold_in, T_w_eco_in_sensor.C_out) annotation (Line(
        points={{112,15.95},{112,35},{140,35}},
                                              color={28,108,200},
      thickness=1));
  connect(T_w_eco_in_sensor.C_in, loopBreaker.C_out) annotation (Line(points={{150,35},{150,34},{180,34},{180,44}},
                                       color={28,108,200},
      thickness=1));
  connect(T_pumpRec_out_sensor.C_out, P_pumpRec_out_sensor.C_in) annotation (
      Line(points={{120,80.5455},{126,80.5455}},                         color={28,108,200},
      thickness=0.5));
  connect(Q_pumpRec_out_sensor.C_out, pumpRec_controlValve.C_in)
    annotation (Line(points={{150,80.5455},{156,80.5455},{156,80.5455},{160,80.5455}},
                                                         color={28,108,200},
      thickness=0.5));
  connect(flue_gas_sink.C_in, P_flue_gas_sink_sensor.C_out)
    annotation (Line(points={{222,219},{222,204}}, color={95,95,95},
      thickness=1));
  connect(sink.C_in, W_ST_out_sensor.C_out)
    annotation (Line(points={{115,280},{101.88,280}},color={244,125,35}));
  connect(combustionChamber.inlet1, Q_fuel_source_sensor.C_out)
    annotation (Line(points={{-442,-10},{-442,-16}}, color={213,213,0}));
  connect(Q_fuel_source_sensor.C_in, P_fuel_source_sensor.C_out) annotation (
      Line(points={{-442,-26},{-442,-30}},                       color={213,213,
          0}));
  connect(P_fuel_source_sensor.C_in, T_fuel_source_sensor.C_out) annotation (
      Line(points={{-442,-40},{-442,-44}},                       color={213,213,
          0}));
  connect(source_fuel.C_out, T_fuel_source_sensor.C_in)
    annotation (Line(points={{-442,-59},{-442,-54}}, color={213,213,0}));
  connect(GT_generator.C_out, W_GT_sensor.C_in)
    annotation (Line(points={{-352.8,60},{-346,60}}, color={244,125,35}));
  connect(W_GT_sensor.C_out, sink_power.C_in)
    annotation (Line(points={{-334.12,60},{-327,60}}, color={244,125,35}));
  connect(GT_generator.C_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-373.92,60},{-382,60},{-382,15}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(W_ST_out_sensor.C_in, ST_generator.C_out)
    annotation (Line(points={{90,280},{77.2,280}},color={244,125,35}));
  connect(HPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{-126,193.44},{-126,193.44},{-126,224},{-126,280},{56.08,280}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPsteamTurbine.C_W_out, ST_generator.C_in) annotation (Line(
      points={{20,253.44},{20,280},{56.08,280}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(pumpRec_controlValve.C_out, loopBreaker.C_in) annotation (Line(points={{172,80.5455},{172,80},{180,80},{180,64}},
                                                                color={28,108,200},
      thickness=0.5));
  connect(pumpRec.C_out, T_pumpRec_out_sensor.C_in) annotation (Line(points={{101,80.5455},{110,80.5455}},
                                               color={28,108,200},
      thickness=0.5));
  connect(AirFilter.C_out, P_filter_out_sensor.C_in)
    annotation (Line(points={{-556,0},{-548,0}},     color={95,95,95},
      thickness=1));
  connect(Q_source_air_sensor.C_out, AirFilter.C_in)
    annotation (Line(points={{-588,0},{-576,0}},     color={95,95,95},
      thickness=1));
  connect(P_filter_out_sensor.C_out, airCompressor.C_in)
    annotation (Line(points={{-536,0},{-524,0}},     color={95,95,95},
      thickness=1));
  connect(HPsuperheater1.C_hot_in, HPsuperheater2.C_hot_out)
    annotation (Line(points={{-186,-1},{-254,-1}},   color={95,95,95},
      thickness=1));
  connect(P_w_HPSH2_out_sensor.C_out, HPST_control_valve.C_in) annotation (Line(
        points={{-280,84},{-280,180},{-203.25,180}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(turbine_P_out_sensor.C_out, HPsuperheater2.C_hot_in)
    annotation (Line(points={{-338,-1},{-292,-1}},   color={95,95,95},
      thickness=1));
  connect(deSH_opening_sensor.Opening, deSH_controlValve.Opening)
    annotation (Line(points={{-165,141.9},{-165,130.182}}, color={0,0,127}));
  connect(Q_deSH_sensor.C_in, loopBreaker.C_in) annotation (Line(points={{-132,120},{180,120},{180,64}},
                              color={28,108,200},
      thickness=0.5));
  connect(Q_deSH_sensor.C_out, deSH_controlValve.C_in) annotation (Line(points={{-144,120},{-152,120},{-152,120},{-158.75,120}},
                                                       color={28,108,200},
      thickness=0.5));
  connect(T_w_HPSH2_out_sensor.C_in, HPsuperheater2.C_cold_out) annotation (
      Line(points={{-280,54},{-280,14.2},{-280.6,14.2}},     color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_HPSH2_out_sensor.C_in, T_w_HPSH2_out_sensor.C_out)
    annotation (Line(points={{-280,72},{-280,66}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(P_w_eco_out_sensor.C_out, Evap_controlValve.C_in) annotation (Line(
        points={{50,34},{46,34},{46,34},{41.25,34}},                 color={28,108,200},
      thickness=1));
  connect(T_w_eco_out_sensor.C_in, Evap_controlValve.C_out) annotation (Line(
        points={{14,34},{22,34},{22,34},{28.75,34}},                 color={28,108,200},
      thickness=1));
  connect(Evap_controlValve.Opening, Evap_opening_sensor.Opening)
    annotation (Line(points={{35,44.1822},{35,59.9}}, color={0,0,127}));
  connect(T_flue_gas_sink_sensor.C_in, economiser.C_hot_out) annotation (Line(points={{170,-1},{124,-1},{124,-0.25}},       color={95,95,95},
      thickness=1));
  connect(T_flue_gas_sink_sensor.C_out, P_flue_gas_sink_sensor.C_in) annotation (Line(points={{182,-1},{222,-1},{222,192}},   color={95,95,95},
      thickness=1));
  connect(P_source_air_sensor.C_in, moistAir_to_FlueGases.outlet) annotation (Line(points={{-636,0},{-652,0}},     color={95,95,95},
      thickness=1));
  connect(P_HPST_out_sensor.C_out, T_HPST_out_sensor.C_in) annotation (Line(points={{-102,180},{-96,180}}, color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(T_HPST_out_sensor.C_out, Reheater.C_cold_in) annotation (Line(points={{-84,180},{-64,180},{-64,30},{-62.6,30},{-62.6,15.8}},
                                                                                                               color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(airCompressor.C_W_in, gasTurbine.C_W_shaft) annotation (Line(
      points={{-496,10.5},{-496,34},{-382,34},{-382,15}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(deSH_controlValve.C_out, HPsuperheater2.C_cold_in) annotation (Line(
        points={{-171.25,120},{-260,120},{-260,34},{-265.4,34},{-265.4,14.2}},
                                                                     color={28,108,200},
      thickness=0.5));
  connect(P_w_HPSH1_out_sensor.C_out, HPsuperheater2.C_cold_in) annotation (
     Line(points={{-214,34},{-265.4,34},{-265.4,14.2}},
                                                color={28,108,200},
      pattern=LinePattern.Dash,
      thickness=1));
  connect(Q_pump_out_sensor.C_out, loopBreaker.C_in)
    annotation (Line(points={{176,157},{180,157},{180,64}},
                                                         color={28,108,200},
      thickness=1));
  connect(economiser.C_cold_out, P_w_eco_out_sensor.C_in) annotation (Line(
        points={{96,15.95},{96,34},{62,34}},       color={28,108,200},
      thickness=1));
  connect(moistAir_to_FlueGases.inlet, H_sensor.C_out) annotation (Line(
      points={{-672,0},{-682,0}},
      color={85,170,255},
      thickness=1));
  connect(H_sensor.C_in, source_air.C_out) annotation (Line(
      points={{-694,0},{-703,0}},
      color={85,170,255},
      thickness=1));
  connect(HRSG_friction.Kfr,HRSG_Kfr)  annotation (Line(points={{50,-4},{50,-20}}, color={0,0,127}));
  connect(evaporator.C_hot_out, HRSG_friction.C_in) annotation (Line(points={{6,-1},{22,-1},{22,0},{40,0}}, color={95,95,95}));
  connect(HRSG_friction.C_out, economiser.C_hot_in) annotation (Line(points={{60,0},{84,0},{84,-0.25}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-720,-120},{260,300}})),
                                                              Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-720,-120},{260,300}}),
        graphics={Rectangle(
          extent={{-324,44},{246,-46}},
          pattern=LinePattern.None,
          lineColor={0,0,0},
          fillColor={158,158,158},
          fillPattern=FillPattern.Solid), Text(
          extent={{-92,-30},{54,-42}},
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
          textString="Desuperheater")}));
end MetroscopiaCCGT_direct;
