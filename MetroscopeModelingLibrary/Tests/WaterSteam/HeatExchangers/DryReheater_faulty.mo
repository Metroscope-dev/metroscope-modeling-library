within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model DryReheater_faulty
  extends DryReheater_direct     (
      dryReheater(faulty = true));

  input Real Fault_fouling(start=0);
  input Real Fault_partition_plate_leak(start=0);
  input Real Fault_tube_rupture_leak(start=0);
  input Real Fault_hot_side_partition_plate_leak( start = 0);

equation

  // Failure definition
  dryReheater.fouling = Fault_fouling + 10*time; // Investigate the fouling fault
  dryReheater.partition_plate_leak = Fault_partition_plate_leak;
  dryReheater.tube_rupture_leak = Fault_tube_rupture_leak;
  dryReheater.hot_side_partition_plate_leak = Fault_hot_side_partition_plate_leak;

end DryReheater_faulty;
