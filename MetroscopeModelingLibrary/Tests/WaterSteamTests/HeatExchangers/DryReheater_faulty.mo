within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model DryReheater_faulty
  extends DryReheater_direct(
      dryReheater(faulty = true));

  Real Failure_fouling(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;

  // Failure definition
  dryReheater.fouling = Failure_fouling;

end DryReheater_faulty;
