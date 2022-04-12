within MetroscopeModelingLibrary.Examples.NuclearPowerPlant;
model MetroscopiaNPP_reverse
  // Boundary Conditions
    // Steam generator
    input Real steam_generator_vapor_fraction(start = 1);
    input Real steam_generator_steam_P_out(start = 50) "barA";
    input Real steam_generator_thermal_power(start = 1880) "MWth";

    // Boundary conditions
    input Real cold_source_P_out(start = 3) "barA";
    input Real cold_source_T_out(start = 15) "°C";
    input Real cold_source_Qv_out(start = 50) "m3/s";

  // Observables used for calibration
    // Steam Generator
    input Real steam_generator_P_in(start=57.782) "barA";

    // HP turbines
    input Real HP_turbine_P_out(start=19.3986) "barA";
    input Real HP_steam_extraction_P_out(start=31.0032) "barA";
    input Real HP_turbine_P_in(start=48.5305) "barA";

    // HP Control Valve
    input Real HP_control_valve_opening(start=0.8);

  // Calibrated parameters
    // HP turbines inlet control valve
    output Units.Cv HP_control_valve_Cvmax;
    output Units.Cv HP_control_valve_Cv;

  // Components
    // Steam Generator
    WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-172,-52},{-128,40}})));
    Sensors.WaterSteam.WaterPressureSensor steam_generator_P_in_sensor annotation (Placement(transformation(extent={{-112,-12},{-124,0}})));
    WaterSteam.BoundaryConditions.Sink blow_down_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-150,-88})));

    // HP
    Sensors.WaterSteam.WaterPressureSensor HP_turbine_P_in_sensor annotation (Placement(transformation(extent={{-106,66.546},{-94,78.546}})));

      // Temporary components
      WaterSteam.BoundaryConditions.Source temp_feedwater_source annotation (Placement(transformation(extent={{172,-16},{152,4}})));
      WaterSteam.BoundaryConditions.Sink temp_main_steam_sink annotation (Placement(transformation(extent={{152,62.546},{172,82.546}})));
  WaterSteam.Pipes.ControlValve HP_control_valve annotation (Placement(transformation(extent={{-136,70},{-126,82}})));
  Sensors.Other.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,92},{-126,102}})));
equation
  // ----- Boundary Conditions ------
  steam_generator.vapor_fraction = steam_generator_vapor_fraction;
  steam_generator.thermal_power = steam_generator_thermal_power * 1e6;
  steam_generator.steam_pressure = steam_generator_steam_P_out * 1e5;

  // ----- Components ------
  // SteamGenerator
    // Observable used for calibration
    steam_generator_P_in_sensor.P_barA = steam_generator_P_in;

    // Purge fixed parameters
    steam_generator.Q_purge = 1e-2;
    steam_generator.P_purge = 50e5;

  // HP systems
    // HP turbines inlet control valve
      // Observables used for calibration (inlet)
      HP_turbine_P_in_sensor.P_barA = HP_turbine_P_in;
      HP_control_valve_opening_sensor.Opening = HP_control_valve_opening;

      // Calibrated parameters
      HP_control_valve.Cvmax = HP_control_valve_Cvmax;
      HP_control_valve.Cv = HP_control_valve_Cv;


  // Temporary components
  temp_feedwater_source.Q_out = -1000;
  connect(steam_generator_P_in_sensor.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{-124,-6},{-139,-6}}, color={28,108,200}));
  connect(steam_generator.purge_outlet, blow_down_sink.C_in) annotation (Line(points={{-150,-51.2333},{-150,-83}}, color={28,108,200}));
  connect(steam_generator_P_in_sensor.C_in, temp_feedwater_source.C_out) annotation (Line(points={{-112,-6},{157,-6}}, color={28,108,200}));
  connect(HP_turbine_P_in_sensor.C_out, temp_main_steam_sink.C_in) annotation (Line(points={{-94,72.546},{157,72.546}},  color={28,108,200}));
  connect(steam_generator.steam_outlet, HP_control_valve.C_in) annotation (Line(points={{-150,40},{-150,72.1818},{-136,72.1818}}, color={28,108,200}));
  connect(HP_control_valve.C_out, HP_turbine_P_in_sensor.C_in) annotation (Line(points={{-126,72.1818},{-116,72.1818},{-116,72.546},{-106,72.546}}, color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,80.9091},{-131,91.9}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,140}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,140}})));
end MetroscopiaNPP_reverse;
