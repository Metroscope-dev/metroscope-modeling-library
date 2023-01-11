within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model NTU_HeatExchanger_condenser

  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;

  MetroscopeModelingLibrary.Power.HeatExchange.NTUHeatExchange NTUHeatExchange(config = "condenser");

equation

    NTUHeatExchange.Kth = 50000;
    NTUHeatExchange.S = 100;

    // Cold Side
    NTUHeatExchange.Q_cold = 500;
    NTUHeatExchange.T_cold_in = 273.15 + 50;
    NTUHeatExchange.Cp_cold = 4168;

    // Hot Side
    NTUHeatExchange.Q_hot = 140;
    NTUHeatExchange.T_hot_in = 273.15 + 180;
    NTUHeatExchange.Cp_hot = 1e10; // the hot fluid is condensing : Cp is infinite.

    annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
end NTU_HeatExchanger_condenser;
