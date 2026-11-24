within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_faulty_connectors
    extends LiqLiqHX_direct_connectors
                           (
      liqLiqHX(faulty = true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;

  // Failure definition
  liqLiqHX.fouling = Fault_fouling;

end LiqLiqHX_faulty_connectors;
