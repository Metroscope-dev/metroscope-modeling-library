within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_faulty
  extends Superheater_direct(
      superheater(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_closed_vent(start=0);
  Real Failure_tube_rupture_leak(start=0);


equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_closed_vent = 0 + 100*time; // Fully closed vent at end of simulation
  Failure_tube_rupture_leak = 5*time;

  // Failure definition
  superheater.fouling = Failure_fouling;
  superheater.closed_vent = Failure_closed_vent;
  superheater.tube_rupture_leak = Failure_tube_rupture_leak;

end Superheater_faulty;
