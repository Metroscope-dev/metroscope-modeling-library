within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_faulty
  extends MetroscopiaNPP_direct(superheater(faulty=true), LP_reheater(faulty=true), HP_reheater(faulty=true), condenser(faulty=true));
  import MetroscopeModelingLibrary.Units;

  // Heat exchangers failures
  input Real Failure_superheater_fouling(start=0);
  input Real Failure_superheater_closed_vent(start=0);
  input Real Failure_condenser_fouling(start=0);
  input Real Failure_condenser_air_intake(start=0);
  input Real Failure_LP_reheater_fouling(start=0);
  input Real Failure_HP_reheater_fouling(start=0);
  input Real Failure_water_level_rise(start=0);

  // Leaks
  input Units.PositiveMassFlowRate Failure_bypass_HP_control_valve_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_HP_turbine_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_LP_turbine_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_superheater_tube_rupture_Q(start=0);
  input Units.PositiveMassFlowRate Failure_HP_reheater_tube_rupture_Q(start=0);
  input Units.PositiveMassFlowRate Failure_LP_reheater_tube_rupture_Q(start=0);
  input Units.PositiveMassFlowRate Failure_HP_reheater_spearator_plate_rupture_Q(start=0);
  input Units.PositiveMassFlowRate Failure_LP_reheater_spearator_plate_rupture_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_superheater_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_HP_turbine_ext_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_LP_reheater_drains_to_condenser_Q(start=0);
  input Units.PositiveMassFlowRate Failure_bypass_HP_reheater_drains_to_condenser_Q(start=0);

  // Bypasses
  WaterSteam.Pipes.Leak bypass_HP_control_valve_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-147,51})));
  WaterSteam.Pipes.Leak bypass_HP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-113,51})));
  WaterSteam.Pipes.Leak bypass_LP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={139,109})));
  WaterSteam.Pipes.Leak superheater_tube_rupture annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=0,origin={55,149})));
  WaterSteam.Pipes.Leak HP_reheater_tube_rupture annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-16,-100})));
  WaterSteam.Pipes.Leak LP_reheater_tube_rupture annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={277,-101})));
  WaterSteam.Pipes.Leak HP_reheater_spearator_plate_rupture annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=90,origin={-17,-49})));
  WaterSteam.Pipes.Leak LP_reheater_spearator_plate_rupture annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=90,origin={293,-49})));
  WaterSteam.Pipes.Leak bypass_superheater_to_condenser annotation (Placement(transformation(extent={{9,-9},{-9,9}},rotation=270,origin={-113,143})));
  WaterSteam.Pipes.Leak bypass_HP_turbine_ext_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=0,origin={-11,49})));
  WaterSteam.Pipes.Leak bypass_LP_reheater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={277,-141})));
  WaterSteam.Pipes.Leak bypass_HP_reheater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-16,-144})));
equation
  // Superheater
  superheater.fouling = Failure_superheater_fouling;
  superheater.closed_vent = Failure_superheater_closed_vent;

  // Condenser
  condenser.fouling = Failure_condenser_fouling;
  condenser.air_intake = Failure_condenser_air_intake;

  // LP reheater
  LP_reheater.fouling = Failure_LP_reheater_fouling;

  // HP reheater
  HP_reheater.fouling = Failure_HP_reheater_fouling;
  HP_reheater.water_level_rise = Failure_water_level_rise;

  // Leaks
  bypass_HP_control_valve_to_condenser.Q = Failure_bypass_HP_control_valve_to_condenser_Q + 1e-3;
  bypass_HP_turbine_to_condenser.Q = Failure_bypass_HP_turbine_to_condenser_Q + 1e-3;
  bypass_LP_turbine_to_condenser.Q = Failure_bypass_LP_turbine_to_condenser_Q + 1e-3;
  superheater_tube_rupture.Q = Failure_superheater_tube_rupture_Q + 1e-3;
  HP_reheater_tube_rupture.Q = Failure_HP_reheater_tube_rupture_Q + 1e-3;
  LP_reheater_tube_rupture.Q = Failure_LP_reheater_tube_rupture_Q + 1e-3;
  HP_reheater_spearator_plate_rupture.Q = Failure_HP_reheater_spearator_plate_rupture_Q + 1e-3;
  LP_reheater_spearator_plate_rupture.Q = Failure_LP_reheater_spearator_plate_rupture_Q + 1e-3;
  bypass_superheater_to_condenser.Q = Failure_bypass_superheater_to_condenser_Q + 1e-3;
  bypass_HP_turbine_ext_to_condenser.Q = Failure_bypass_HP_turbine_ext_to_condenser_Q + 1e-3;
  bypass_LP_reheater_drains_to_condenser.Q = Failure_bypass_LP_reheater_drains_to_condenser_Q + 1e-3;
  bypass_HP_reheater_drains_to_condenser.Q = Failure_bypass_HP_reheater_drains_to_condenser_Q + 1e-3;

  connect(superheater_tube_rupture.C_in, superheater.C_hot_in) annotation (Line(points={{46,149},{30,149},{30,112.2},{56,112.2}}, color={217,67,180}));
  connect(bypass_HP_control_valve_to_condenser.C_in, HP_control_valve.C_in) annotation (Line(points={{-147,60},{-148,60},{-148,72.1818},{-136,72.1818}}, color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser.C_in, HP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{-113,60},{-112,60},{-112,72.1818},{-106,72.1818}}, color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser.C_out, bypass_HP_control_valve_to_condenser.C_out) annotation (Line(points={{-113,42},{-113,32},{-147,32},{-147,42}}, color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-113,42},{-113,32},{320,32},{320,100},{392,100},{392,76}}, color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_in, LP_turbine_1.C_in) annotation (Line(points={{139,118},{140,118},{140,130.182},{152,130.182}}, color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{139,100},{140,100},{140,86},{158,86},{158,32},{320,32},{320,100},{392,100},{392,76}}, color={217,67,180}));
  connect(superheater_tube_rupture.C_out, superheater.C_cold_out) annotation (Line(points={{64,149},{72,149},{72,130},{71.8,130},{71.8,120}}, color={217,67,180}));
  connect(HP_pump_P_out_sensor.C_out, HP_reheater_tube_rupture.C_in) annotation (Line(points={{-6,-70},{-16,-70},{-16,-91}}, color={217,67,180}));
  connect(LP_reheater_tube_rupture.C_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{277,-110},{276,-110},{276,-119.818},{288,-119.818}}, color={217,67,180}));
  connect(LP_reheater_tube_rupture.C_in, LP_reheater.C_cold_in) annotation (Line(points={{277,-92},{276,-92},{276,-84},{294,-84},{294,-70},{284.2,-70}}, color={217,67,180}));
  connect(HP_reheater_spearator_plate_rupture.C_in, HP_reheater.C_cold_in) annotation (Line(points={{-17,-58},{-16,-58},{-16,-70},{-19.8,-70}}, color={217,67,180}));
  connect(HP_reheater_spearator_plate_rupture.C_out, HP_reheater_P_cold_out_sensor.C_in) annotation (Line(points={{-17,-40},{-16,-40},{-16,-34},{-56,-34},{-56,-70},{-60,-70}}, color={217,67,180}));
  connect(LP_reheater_spearator_plate_rupture.C_in, LP_reheater.C_cold_in) annotation (Line(points={{293,-58},{294,-58},{294,-70},{284.2,-70}}, color={217,67,180}));
  connect(LP_reheater_spearator_plate_rupture.C_out, LP_reheater_P_cold_out_sensor.C_in) annotation (Line(points={{293,-40},{292,-40},{292,-34},{246,-34},{246,-70},{242,-70}}, color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_in, superheater_control_valve.C_out) annotation (Line(points={{-113,134},{-113,112.182},{-126,112.182}}, color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-113,152},{-112,152},{-112,192},{392,192},{392,76}}, color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-2,49},{4,49},{4,32},{320,32},{320,100},{392,100},{392,76}}, color={217,67,180}));
  connect(HP_reheater_tube_rupture.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-16,-109},{-16,-121.818},{6,-121.818}}, color={217,67,180}));
  connect(bypass_LP_reheater_drains_to_condenser.C_in, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{277,-132},{276,-132},{276,-119.818},{288,-119.818}}, color={217,67,180}));
  connect(bypass_LP_reheater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{277,-150},{278,-150},{278,-162},{472,-162},{472,100},{392,100},{392,76}}, color={217,67,180}));
  connect(bypass_HP_reheater_drains_to_condenser.C_in, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-16,-135},{-16,-121.818},{6,-121.818}}, color={217,67,180}));
  connect(bypass_HP_reheater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-16,-153},{-16,-162},{472,-162},{472,100},{392,100},{392,76}}, color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_in, HP_turbines_ext_P_sensor.C_in) annotation (Line(points={{-20,49},{-24,49},{-24,62},{-36,62},{-36,60}}, color={217,67,180}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MetroscopiaNPP_faulty;
