within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model NTU_HeatExchanger_monophasic_cross_current
   extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;

   MetroscopeModelingLibrary.Power.HeatExchange.NTUHeatExchange NTUHeatExchange(config = "monophasic_cross_current",QCp_max_side = "hot");

equation
    NTUHeatExchange.Kth = 1000;
    NTUHeatExchange.S = 100;

    // Cold Side
    NTUHeatExchange.Q_cold = 11;
    NTUHeatExchange.T_cold_in = 273.15 + 200;
    NTUHeatExchange.Cp_cold = 2066;

    // Hot Side
    NTUHeatExchange.Q_hot = 590;
    NTUHeatExchange.T_hot_in = 273.15 + 293;
    NTUHeatExchange.Cp_hot = 1105;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end NTU_HeatExchanger_monophasic_cross_current;
