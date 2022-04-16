within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model Condenser_faulty
  extends Condenser_direct(
      condenser(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_air_intake(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_air_intake = 0 + 1e-3 * time;

  // Failure definition
  condenser.fouling = Failure_fouling;
  condenser.air_intake = Failure_air_intake;

end Condenser_faulty;
