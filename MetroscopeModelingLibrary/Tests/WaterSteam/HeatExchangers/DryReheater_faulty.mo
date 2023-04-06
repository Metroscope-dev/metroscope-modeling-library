within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model DryReheater_faulty
  extends DryReheater_direct(
      dryReheater(faulty = true));

  Real Failure_fouling(start=0);
  Real Failure_partition_plate_leak(start=0);
  Real Failure_tube_rupture_leak(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_partition_plate_leak = 0 + 10*time;
  Failure_tube_rupture_leak = 0 + 10*time;

  // Failure definition
  dryReheater.fouling = Failure_fouling;
  dryReheater.partition_plate_leak = Failure_partition_plate_leak;
  dryReheater.tube_rupture_leak = Failure_tube_rupture_leak;

end DryReheater_faulty;
