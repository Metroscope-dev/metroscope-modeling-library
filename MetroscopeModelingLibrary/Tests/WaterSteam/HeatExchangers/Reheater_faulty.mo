within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_faulty

  extends Reheater_direct(reheater(faulty=true));

  Real Fault_fouling(start=0);
  Real Fault_water_level_rise(start=0);
  Real Fault_partition_plate_leak(start=0);
  Real Fault_tube_rupture_leak(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;
  Fault_water_level_rise = 0 - 0.1*time;
  Fault_partition_plate_leak = 0 + 10*time;
  Fault_tube_rupture_leak = 0 + 5*time;

  // Failure definition
  reheater.fouling = Fault_fouling;
  reheater.water_level_rise = Fault_water_level_rise;
  reheater.partition_plate_leak = Fault_partition_plate_leak;
  reheater.tube_rupture_leak = Fault_tube_rupture_leak;

  annotation (Icon(graphics={
        Ellipse(lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,0,255},
                fillColor={28,108,200},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-40,60},{60,0},{-40,-60},{-40,60}})}));
end Reheater_faulty;
