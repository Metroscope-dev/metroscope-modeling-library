within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Filter_faulty
  extends Filter_direct(filter(faulty=true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 1*time;

  // Failure definition
  filter.fouling = Fault_fouling;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Filter_faulty;
