within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Condenser_faulty
  extends Condenser_direct(
      condenser(faulty = true));

  Real Fault_fouling(start=0);
  Real Fault_air_intake(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;
  Fault_air_intake = 0 + 1e-3 * time;

  // Failure definition
  condenser.fouling = Fault_fouling;
  condenser.air_intake = Fault_air_intake;

end Condenser_faulty;
