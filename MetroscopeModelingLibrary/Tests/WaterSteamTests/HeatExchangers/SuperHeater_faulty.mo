within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model SuperHeater_faulty
  extends SuperHeater_direct(
      superheater(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_closed_vent(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_closed_vent = 0 + 100*time; // Fully closed vent at end of simulation

  // Failure definition
  superheater.fouling = Failure_fouling;
  superheater.closed_vent = Failure_closed_vent;

end SuperHeater_faulty;
