within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model AirCompressor_faulty
  extends AirCompressor_direct(airCompressor(faulty=true));

  Real Fault_eta_is_decrease(start=0);
  Real Fault_tau_decrease(start=0);

equation

  // Failure input
  Fault_eta_is_decrease = 0 + 10*time;
  Fault_tau_decrease = 0 + 10*time;

  // Failure definition
  airCompressor.eta_is_decrease = Fault_eta_is_decrease;
  airCompressor.tau_decrease = Fault_tau_decrease;

end AirCompressor_faulty;
