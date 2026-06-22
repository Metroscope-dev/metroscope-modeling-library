within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_faulty
  extends Superheater_direct     (
      superheater(faulty = true));

  input Real Fault_fouling_or_drains_flooding(start=0); // Modeled as fouling, but represents either fouling drains flooding in on the hot-side
  input Real Fault_closed_vent(start=0);
  input Real Fault_tube_rupture_leak(start=0);

equation

  // Failure definition
  superheater.fouling_or_drains_flooding = Fault_fouling_or_drains_flooding + 10*time; // Study fouling/drains flooding fault
  superheater.closed_vent = Fault_closed_vent;
  superheater.tube_rupture_leak = Fault_tube_rupture_leak;

end Superheater_faulty;
