within MetroscopeModelingLibrary.Tests.Power.HeatExchange;
model CondenserCounterCurrentNTUHXTest

  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;

  MetroscopeModelingLibrary.Power.HeatExchange.NTUHeatExchange NTUHeatExchange(config = "condenser_counter_current");

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

    annotation (Placement(transformation(extent={{-10,-10},{10,10}})),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CondenserCounterCurrentNTUHXTest;
