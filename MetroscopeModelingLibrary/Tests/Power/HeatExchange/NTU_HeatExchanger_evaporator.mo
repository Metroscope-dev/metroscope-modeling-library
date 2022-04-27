within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model NTU_HeatExchanger_evaporator
    extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;

  MetroscopeModelingLibrary.Power.HeatExchange.NTUHeatExchange NTUHeatExchange(config = "evaporator");

equation

    NTUHeatExchange.Kth = 102000;
    NTUHeatExchange.S = 10;

    // Cold Side
    NTUHeatExchange.Q_cold = 100;
    NTUHeatExchange.T_cold_in = 273.15 + 130;
    NTUHeatExchange.Cp_cold = 1e10; // vaporisation in progress : Cp is infinite

    // Hot Side
    NTUHeatExchange.Q_hot = 580;
    NTUHeatExchange.T_hot_in = 273.15 + 190;
    NTUHeatExchange.Cp_hot =1082; // (J/(kg.K))
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end NTU_HeatExchanger_evaporator;
