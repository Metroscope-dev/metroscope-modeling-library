within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Condenser_faulty
  extends Condenser_direct     (
      condenser(faulty = true));

  input Real Fault_fouling(start=0);
  input Real Fault_air_intake(start=0);
  input Real Fault_Qv_cold_in_decrease(start=0);

equation

  // Failure definition
  condenser.fouling = Fault_fouling + 10*time; // Study the fouling fault
  condenser.air_intake = Fault_air_intake;
  condenser.Qv_cold_in_decrease = Fault_Qv_cold_in_decrease;

end Condenser_faulty;
