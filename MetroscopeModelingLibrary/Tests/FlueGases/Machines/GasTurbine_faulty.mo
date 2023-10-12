within MetroscopeModelingLibrary.Tests.FlueGases.Machines;
model GasTurbine_faulty
  extends GasTurbine_direct(gasTurbine(faulty=true));

  Real Fault_eta_is_decrease(start=0);

equation

  // Failure input
  Fault_eta_is_decrease = 0 + 10*time;

  // Failure definition
  gasTurbine.eta_is_decrease = Fault_eta_is_decrease;

end GasTurbine_faulty;
