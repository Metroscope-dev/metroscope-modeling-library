within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_faulty
  extends Reheater_direct(
      reheater(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_water_level_rise(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_water_level_rise = 0 - 0.1*time;

  // Failure definition
  reheater.fouling = Failure_fouling;
  reheater.water_level_rise = Failure_water_level_rise;

end Reheater_faulty;
