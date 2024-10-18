within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
model Filter_faulty
  extends Filter_direct(filter(faulty=true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;

  // Failure definition
  filter.fouling = Fault_fouling;

end Filter_faulty;
