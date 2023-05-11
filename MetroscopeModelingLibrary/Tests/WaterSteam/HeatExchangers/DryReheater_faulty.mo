within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model DryReheater_faulty
  extends DryReheater_direct(
      dryReheater(faulty = true));

  Real Fault_fouling(start=0);
  Real Fault_partition_plate_leak(start=0);
  Real Fault_tube_rupture_leak(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;
  Fault_partition_plate_leak = 0 + 10*time;
  Fault_tube_rupture_leak = 0 + 10*time;

  // Failure definition
  dryReheater.fouling = Fault_fouling;
  dryReheater.partition_plate_leak = Fault_partition_plate_leak;
  dryReheater.tube_rupture_leak = Fault_tube_rupture_leak;

end DryReheater_faulty;
