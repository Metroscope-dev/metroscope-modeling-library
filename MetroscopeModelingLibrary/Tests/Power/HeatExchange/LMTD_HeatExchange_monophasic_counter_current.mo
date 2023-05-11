within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model LMTD_HeatExchange_monophasic_counter_current
   extends MetroscopeModelingLibrary.Utilities.Icons.Tests.PowerTestIcon;

   MetroscopeModelingLibrary.Power.HeatExchange.LMTDHeatExchange LMTDHeatExchange;
equation
    LMTDHeatExchange.Kth = 1000;
    LMTDHeatExchange.S = 100;

    // Cold Side
    LMTDHeatExchange.T_cold_in = 273.15 + 30;
    LMTDHeatExchange.T_cold_out = 273.15 + 200;

    // Hot Side
    LMTDHeatExchange.T_hot_in = 273.15 + 230;
    LMTDHeatExchange.T_hot_out = 273.15 + 80;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end LMTD_HeatExchange_monophasic_counter_current;
