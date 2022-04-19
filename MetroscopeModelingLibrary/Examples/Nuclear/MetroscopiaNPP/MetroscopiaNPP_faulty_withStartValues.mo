within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_faulty_withStartValues
  extends MetroscopiaNPP_faulty(
    HP_control_valve(DP(start=-149999.640625)),
    HP_pump(DP(start=5300125.5), h_out(start=316239.46875)),
    HP_reheater(
      C_hot_out(h_outflow(start=295555.46875)),
      cold_side_condensing(h_in(start=502807.03125)),
      cold_side_pipe(DP(start=-100000.140625)),
      h_vap_sat(start=2803283.5),
      hot_side_pipe(h_in(start=2549398.5))),
    HP_reheater_drains_control_valve(DP(start=-199992.625)),
    HP_turbine_1(P_out(start=3100009.0)),
    HP_turbine_2(
      P_out(start=1940005.0),
      h_in(start=2732885.75),
      xm(start=0.951332151889801)),
    HP_turbines_ext(x_ext_out(start=0.9605896472930908)),
    LP_pump(
      DP(start=683263.375),
      Qv_in(start=1.0661767721176147),
      h_out(start=163942.015625)),
    LP_reheater(
      C_cold_out(h_outflow(start=293503.3125)),
      cold_side_deheating(h_in(start=293503.3125)),
      cold_side_pipe(DP(start=-100001.7890625)),
      h_vap_sat(start=2748107.75)),
    LP_reheater_drains_control_valve(DP(start=-99978.4765625)),
    LP_turbine_1(P_out(start=500003.4375)),
    LP_turbine_2(
      P_out(start=6982.64013671875),
      h_in(start=2711326.0),
      xm(start=0.9541946053504944)),
    LP_turbines_ext(x_ext_out(start=0.9825506806373596)),
    LP_turbines_ext_P_sensor(h_in(start=2711326.0)),
    condenser(
      C_cold_out(h_outflow(start=108142.765625)),
      C_hot_out(P(start=16716.490234375), h_outflow(start=163149.5625)),
      P_incond(start=-5.202058929970717E-25),
      Q_cold(start=49905.9375),
      cold_side_pipe(DP(start=-99784.9375))),
    flash_tank_inlet_pipe(P_out(start=552024.4375)),
    flash_tank_outlet_pipe(DP(start=47800.56640625)),
    superheater(
      C_cold_out(h_outflow(start=2847863.0)),
      Q_hot(start=44.2562141418457),
      cold_side_deheating(h_in(start=2847863.0)),
      h_vap_sat_cold(start=2797730.5),
      h_vap_sat_hot(start=2800897.75),
      hot_side_condensing(h_out(start=1551314.375)),
      hot_side_pipe(DP(start=-100001.0859375))),
    superheater_control_valve(DP(start=-900088.75)));
end MetroscopiaNPP_faulty_withStartValues;
