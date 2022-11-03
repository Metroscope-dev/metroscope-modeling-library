within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_faulty
  extends Reheater_direct(
      reheater(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_water_level_rise(start=0);
  Real Failure_separating_plate_leak(start=0);
  Real Failure_tube_rupture_leak(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_water_level_rise = 0 - 0.1*time;
  Failure_separating_plate_leak =  0 + 10*time;
  Failure_tube_rupture_leak = 0 + 5*time;

  // Failure definition
  reheater.fouling = Failure_fouling;
  reheater.water_level_rise = Failure_water_level_rise;
  reheater.separating_plate_leak = Failure_separating_plate_leak;
  reheater.tube_rupture_leak = Failure_tube_rupture_leak;

end Reheater_faulty;
