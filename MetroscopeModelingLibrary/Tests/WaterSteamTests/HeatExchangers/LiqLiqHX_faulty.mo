within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model LiqLiqHX_faulty
  extends LiqLiqHX_direct(
      liqLiqHX(faulty = true));

  Real Failure_fouling(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;

  // Failure definition
  liqLiqHX.fouling = Failure_fouling;

end LiqLiqHX_faulty;
