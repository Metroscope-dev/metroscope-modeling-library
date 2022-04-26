within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model NTU_HeatExchange_evaporator
  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;

  MetroscopeModelingLibrary.Power.HeatExchange.NTUHeatExchange NTUHeatExchange(config = "evaporator");
equation
    NTUHeatExchange.Kth = 1000;
    NTUHeatExchange.S = 100;

    // Cold Side
    NTUHeatExchange.Q_cold = 1000;
    NTUHeatExchange.T_cold_in = 273.15 + 100;
    NTUHeatExchange.Cp_cold = 4.2;

    // Hot Side
    NTUHeatExchange.Q_hot = 50;
    NTUHeatExchange.T_hot_in = 273.15 + 200;
    NTUHeatExchange.Cp_hot = 3;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end NTU_HeatExchange_evaporator;
