within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_faulty
    extends LiqLiqHX_direct(
      liqLiqHX(faulty = true));

  input Real Fault_fouling(start=0);
  input Real Fault_tube_rupture( start = 0);

equation

  // Failure definition
  liqLiqHX.fouling = Fault_fouling + 10*time; // Study the fouling fault
  liqLiqHX.tube_rupture_leak = Fault_tube_rupture;

end LiqLiqHX_faulty;
