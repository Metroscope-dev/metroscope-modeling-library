within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model Metroscopia_NPP_direct_with_Start_Values
  extends Metroscopia_NPP_direct(
    HP_control_valve(DP(start=-150000.10398137878)),
    HP_pump(DP(start=5300155.457528055)),
    HP_reheater(
      C_hot_out(h_outflow(start=295535.5112744119)),
      h_vap_sat(start=2803283.6257333425),
      hot_side_pipe(h_in(start=2549399.1448339843))),
    HP_reheater_drains_control_valve(DP(start=-200000.31734817117)),
    HP_turbine_1(P_out(start=3099998.45779697)),
    HP_turbine_2(
      P_out(start=1939998.7488344882),
      h_in(start=2732885.398891378),
      xm(start=0.9513320483434197)),
    HP_turbines_ext(x_ext_out(start=0.9605894819869373)),
    LP_pump(
      DP(start=683287.4823394117),
      Qv_in(start=1.0661631683048423),
      h_out(start=163909.76365609127)),
    LP_reheater(
      C_cold_out(h_outflow(start=293480.4640605938)),
      cold_side_deheating(h_in(start=293480.4640605938)),
      h_vap_sat(start=2748107.4815082387)),
    LP_reheater_drains_control_valve(DP(start=-100001.61233259972)),
    LP_turbine_1(P_out(start=499999.4958561882)),
    LP_turbine_2(
      P_out(start=6979.743676012867),
      h_in(start=2711326.868510008),
      xm(start=0.9541932851606466)),
    LP_turbines_ext(x_ext_out(start=0.9825512482992033)),
    LP_turbines_ext_P_sensor(h_in(start=2711326.8685100074)),
    cold_source(h_out(start=63182.42669967564)),
    condenser(
      C_cold_out(h_outflow(start=108089.31410937822)),
      C_hot_out(P(start=16713.621650359262), h_outflow(start=163117.28311614168)),
      Q_cold(start=49964.95912499571)),
    flash_tank_inlet_pipe(P_out(start=552047.3927557372)),
    steam_dryer(C_hot_steam(h_outflow(start=2778769.130646904))),
    superheater(
      C_cold_out(h_outflow(start=2847864.331394589)),
      Q_hot(start=44.25704827823427),
      cold_side_deheating(h_in(start=2847864.331394589)),
      h_vap_sat_cold(start=2797730.36994302),
      h_vap_sat_hot(start=2800897.3791935025),
      hot_side_condensing(h_out(start=1551308.1818043843))),
    superheater_control_valve(DP(start=-900029.3313351283)));
end Metroscopia_NPP_direct_with_Start_Values;
