within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_faulty

  extends MetroscopiaNPP_direct_withStartValues(
    superheater(faulty=true),
    condenser(faulty=true),
    LP_heater(faulty=true),
    HP_heater(faulty=true),
    steam_generator(faulty=true));

  import MetroscopeModelingLibrary.Utilities.Units;

  // Heat exchangers failures
  input Real Fault_superheater_fouling(start=0);
  input Real Fault_superheater_closed_vent(start=0);
  input Real Fault_condenser_fouling(start=0);
  input Real Fault_condenser_air_intake(start=0);
  input Real Fault_condenser_Qv_cold_in_decrease(start=0);
  input Real Fault_LP_heater_fouling(start=0);
  input Real Fault_HP_heater_fouling(start=0);
  input Real Fault_water_level_rise(start=0);
  input Real Fault_mass_flow_bias(start=0);
  input Real Fault_HP_heater_tube_rupture_Q(start=0);
  input Real Fault_HP_heater_partition_plate_rupture_Q(start=0);
  input Real Fault_LP_heater_tube_rupture_Q(start=0);
  input Real Fault_LP_heater_partition_plate_rupture_Q(start=0);

  // Leaks
  input Real Fault_bypass_HP_control_valve_to_condenser_Q(start=0);
  input Real Fault_bypass_HP_turbine_to_condenser_Q(start=0);
  input Real Fault_bypass_LP_turbine_to_condenser_Q(start=0);
  input Real Fault_superheater_tube_rupture_Q(start=0);
  input Real Fault_bypass_superheater_to_condenser_Q(start=0);
  input Real Fault_bypass_HP_turbine_ext_to_condenser_Q(start=0);
  input Real Fault_bypass_LP_heater_drains_to_condenser_Q(start=0);
  input Real Fault_bypass_HP_heater_drains_to_condenser_Q(start=0);

  // Bypasses
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_control_valve_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-147,51})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-113,51})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_LP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={139,109})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_superheater_to_condenser annotation (Placement(transformation(extent={{9,-9},{-9,9}},rotation=270,origin={-113,143})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_turbine_ext_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=0,origin={-11,49})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_LP_heater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={277,-141})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_heater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-16,-144})));
equation

  // Mass flow bias
  steam_generator.feed_water_flow_rate_measurement_bias = Fault_mass_flow_bias;

  // Superheater
  superheater.fouling = Fault_superheater_fouling;
  superheater.closed_vent = Fault_superheater_closed_vent;
  superheater.tube_rupture.Q = Fault_superheater_tube_rupture_Q + 1e-3;

  // Condenser
  condenser.fouling = Fault_condenser_fouling;
  condenser.air_intake = Fault_condenser_air_intake;
  condenser.Qv_cold_in_decrease = Fault_condenser_Qv_cold_in_decrease;

  // LP reheater
  LP_heater.fouling = Fault_LP_heater_fouling;
  LP_heater.tube_rupture_leak = Fault_LP_heater_tube_rupture_Q;
  LP_heater.partition_plate_leak = Fault_LP_heater_partition_plate_rupture_Q;

  // HP reheater
  HP_heater.fouling = Fault_HP_heater_fouling;
  HP_heater.water_level_rise = Fault_water_level_rise;
  HP_heater.tube_rupture_leak = Fault_LP_heater_tube_rupture_Q;
  HP_heater.partition_plate_leak = Fault_LP_heater_partition_plate_rupture_Q;

  // Leaks
  bypass_HP_control_valve_to_condenser.Q = Fault_bypass_HP_control_valve_to_condenser_Q + 1e-3;
  bypass_HP_turbine_to_condenser.Q = Fault_bypass_HP_turbine_to_condenser_Q + 1e-3;
  bypass_LP_turbine_to_condenser.Q = Fault_bypass_LP_turbine_to_condenser_Q + 1e-3;
  bypass_superheater_to_condenser.Q = Fault_bypass_superheater_to_condenser_Q + 1e-3;
  bypass_HP_turbine_ext_to_condenser.Q = Fault_bypass_HP_turbine_ext_to_condenser_Q + 1e-3;
  bypass_LP_heater_drains_to_condenser.Q = Fault_bypass_LP_heater_drains_to_condenser_Q + 1e-3;
  bypass_HP_heater_drains_to_condenser.Q = Fault_bypass_HP_heater_drains_to_condenser_Q + 1e-3;

  connect(bypass_HP_turbine_to_condenser.C_out, bypass_HP_control_valve_to_condenser.C_out) annotation (Line(points={{-113,42},{-113,32},{-147,32},{-147,42}}, color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-113,42},{-113,32},{320,32},{320,100},{392.5,100},{392.5,74.2864}},
                                                                                                                                                         color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{139,100},{140,100},{140,86},{158,86},{158,32},{320,32},{320,100},{392.5,100},{392.5,74.2864}},
                                                                                                                                                                                    color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_in, superheater_control_valve.C_out) annotation (Line(points={{-113,134},{-113,112.182},{-126,112.182}}, color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-113,152},{-112,152},{-112,192},{392.5,192},{392.5,74.2864}},
                                                                                                                                                    color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-2,49},{4,49},{4,32},{320,32},{320,100},{392.5,100},{392.5,74.2864}},
                                                                                                                                                               color={217,67,180}));
  connect(bypass_LP_heater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{277,-150},{278,-150},{278,-162},{472,-162},{472,100},{392.5,100},{392.5,74.2864}},
                                                                                                                                                                                color={217,67,180}));
  connect(bypass_HP_heater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-16,-153},{-16,-162},{472,-162},{472,100},{392.5,100},{392.5,74.2864}},
                                                                                                                                                                     color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_in, LPT1.C_in) annotation (Line(points={{139,118},{140,118},{140,130},{151,130}}, color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_in, HP_extract_P_sensor.C_in) annotation (Line(points={{-20,49},{-28,49},{-28,58},{-40,58},{-40,54}}, color={28,108,200}));
  connect(bypass_HP_control_valve_to_condenser.C_in, HP_control_valve.C_in) annotation (Line(points={{-147,60},{-148,60},{-148,72},{-135,72}}, color={28,108,200}));
  connect(bypass_HP_turbine_to_condenser.C_in, HPT_P_in_sensor.C_in) annotation (Line(points={{-113,60},{-113,72},{-106,72}}, color={28,108,200}));
  connect(bypass_LP_heater_drains_to_condenser.C_in, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{277,-132},{276,-132},{276,-119.818},{288,-119.818}}, color={217,67,180}));
  connect(bypass_HP_heater_drains_to_condenser.C_in, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-16,-135},{-16,-121.818},{6,-121.818}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MetroscopiaNPP_faulty;
