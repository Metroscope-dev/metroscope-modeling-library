within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_faulty

  extends MetroscopiaNPP_direct_withStartValues(
    superheater(faulty=true),
    condenser(faulty=true),
    LP_heater(faulty=true),
    HP_heater(faulty=true),
    Q_feedwater_sensor(faulty=true));

  import MetroscopeModelingLibrary.Units;

  // Heat exchangers failures
  input Real Failure_superheater_fouling(start=0);
  input Real Failure_superheater_closed_vent(start=0);
  input Real Failure_condenser_fouling(start=0);
  input Real Failure_condenser_air_intake(start=0);
  input Real Failure_LP_heater_fouling(start=0);
  input Real Failure_HP_heater_fouling(start=0);
  input Real Failure_water_level_rise(start=0);
  input Real Failure_mass_flow_bias(start=0);
  input Real Failure_HP_heater_tube_rupture_Q(start=0);
  input Real Failure_HP_heater_separator_plate_rupture_Q(start=0);
  input Real Failure_LP_heater_tube_rupture_Q(start=0);
  input Real Failure_LP_heater_separator_plate_rupture_Q(start=0);

  // Leaks
  input Real Failure_bypass_HP_control_valve_to_condenser_Q(start=0);
  input Real Failure_bypass_HP_turbine_to_condenser_Q(start=0);
  input Real Failure_bypass_LP_turbine_to_condenser_Q(start=0);
  input Real Failure_superheater_tube_rupture_Q(start=0);
  input Real Failure_bypass_superheater_to_condenser_Q(start=0);
  input Real Failure_bypass_HP_turbine_ext_to_condenser_Q(start=0);
  input Real Failure_bypass_LP_heater_drains_to_condenser_Q(start=0);
  input Real Failure_bypass_HP_heater_drains_to_condenser_Q(start=0);

  // Bypasses
  WaterSteam.Pipes.Leak bypass_HP_control_valve_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-147,51})));
  WaterSteam.Pipes.Leak bypass_HP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-113,51})));
  WaterSteam.Pipes.Leak bypass_LP_turbine_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={139,109})));
  WaterSteam.Pipes.Leak bypass_superheater_to_condenser annotation (Placement(transformation(extent={{9,-9},{-9,9}},rotation=270,origin={-113,143})));
  WaterSteam.Pipes.Leak bypass_HP_turbine_ext_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=0,origin={-11,49})));
  WaterSteam.Pipes.Leak bypass_LP_heater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={277,-141})));
  WaterSteam.Pipes.Leak bypass_HP_heater_drains_to_condenser annotation (Placement(transformation(extent={{-9,-9},{9,9}},rotation=270,origin={-16,-144})));
equation

  // Mass flow bias
  Q_feedwater_sensor.mass_flow_rate_bias = Failure_mass_flow_bias;

  // Superheater
  superheater.fouling = Failure_superheater_fouling;
  superheater.closed_vent = Failure_superheater_closed_vent;
  superheater.tube_rupture.Q = Failure_superheater_tube_rupture_Q + 1e-3;

  // Condenser
  condenser.fouling = Failure_condenser_fouling;
  condenser.air_intake = Failure_condenser_air_intake;

  // LP reheater
  LP_heater.fouling = Failure_LP_heater_fouling;
  LP_heater.tube_rupture_leak = Failure_LP_heater_tube_rupture_Q;
  LP_heater.separating_plate_leak = Failure_LP_heater_separator_plate_rupture_Q;

  // HP reheater
  HP_heater.fouling = Failure_HP_heater_fouling;
  HP_heater.water_level_rise = Failure_water_level_rise;
  HP_heater.tube_rupture_leak = Failure_LP_heater_tube_rupture_Q;
  HP_heater.separating_plate_leak = Failure_LP_heater_separator_plate_rupture_Q;

  // Leaks
  bypass_HP_control_valve_to_condenser.Q = Failure_bypass_HP_control_valve_to_condenser_Q + 1e-3;
  bypass_HP_turbine_to_condenser.Q = Failure_bypass_HP_turbine_to_condenser_Q + 1e-3;
  bypass_LP_turbine_to_condenser.Q = Failure_bypass_LP_turbine_to_condenser_Q + 1e-3;
  bypass_superheater_to_condenser.Q = Failure_bypass_superheater_to_condenser_Q + 1e-3;
  bypass_HP_turbine_ext_to_condenser.Q = Failure_bypass_HP_turbine_ext_to_condenser_Q + 1e-3;
  bypass_LP_heater_drains_to_condenser.Q = Failure_bypass_LP_heater_drains_to_condenser_Q + 1e-3;
  bypass_HP_heater_drains_to_condenser.Q = Failure_bypass_HP_heater_drains_to_condenser_Q + 1e-3;

  connect(bypass_HP_turbine_to_condenser.C_out, bypass_HP_control_valve_to_condenser.C_out) annotation (Line(points={{-112.82,42},{-112.82,32},{-146.82,32},{-146.82,42}},
                                                                                                                                                               color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-112.82,42},{-112.82,32},{320,32},{320,100},{392.5,100},{392.5,74}},
                                                                                                                                                         color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{139.18,100},{140,100},{140,86},{158,86},{158,32},{320,32},{320,100},{392.5,100},{392.5,74}},
                                                                                                                                                                                    color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_in, superheater_control_valve.C_out) annotation (Line(points={{-113,134},{-113,112.182},{-126,112.182}}, color={217,67,180}));
  connect(bypass_superheater_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-112.82,152},{-112,152},{-112,192},{392.5,192},{392.5,74}},
                                                                                                                                                    color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-2,49.18},{4,49.18},{4,32},{320,32},{320,100},{392.5,100},{392.5,74}},
                                                                                                                                                               color={217,67,180}));
  connect(bypass_LP_heater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{277.18,-150},{278,-150},{278,-162},{472,-162},{472,100},{392.5,100},{392.5,74}},
                                                                                                                                                                                color={217,67,180}));
  connect(bypass_HP_heater_drains_to_condenser.C_out, condenser.C_hot_in) annotation (Line(points={{-15.82,-153},{-15.82,-162},{472,-162},{472,100},{392.5,100},{392.5,74}},
                                                                                                                                                                     color={217,67,180}));
  connect(bypass_LP_turbine_to_condenser.C_in, LPT1.C_in) annotation (Line(points={{139,118},{140,118},{140,130},{151,130}}, color={217,67,180}));
  connect(bypass_HP_turbine_ext_to_condenser.C_in, HP_extract_P_sensor.C_in) annotation (Line(points={{-20,49},{-28,49},{-28,58},{-40,58},{-40,54}}, color={28,108,200}));
  connect(bypass_HP_control_valve_to_condenser.C_in, HP_control_valve.C_in) annotation (Line(points={{-147,60},{-148,60},{-148,72},{-135,72}}, color={28,108,200}));
  connect(bypass_HP_turbine_to_condenser.C_in, HPT_P_in_sensor.C_in) annotation (Line(points={{-113,60},{-113,72},{-106,72}}, color={28,108,200}));
  connect(bypass_LP_heater_drains_to_condenser.C_in, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{277,-132},{276,-132},{276,-119.818},{288,-119.818}}, color={217,67,180}));
  connect(bypass_HP_heater_drains_to_condenser.C_in, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-16,-135},{-16,-121.818},{6,-121.818}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MetroscopiaNPP_faulty;
