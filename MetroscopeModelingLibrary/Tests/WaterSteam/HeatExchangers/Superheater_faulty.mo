within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_faulty
  extends Superheater_direct(
      superheater(faulty = true));

  Real Fault_fouling(start=0);
  Real Fault_closed_vent(start=0);
  Real Fault_tube_rupture_leak(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;
  Fault_closed_vent = 0 + 100*time; // Fully closed vent at end of simulation
  Fault_tube_rupture_leak = 5*time;

  // Failure definition
  superheater.fouling = Fault_fouling;
  superheater.closed_vent = Fault_closed_vent;
  superheater.tube_rupture_leak = Fault_tube_rupture_leak;

end Superheater_faulty;
