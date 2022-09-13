within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Filter_faulty
  extends Filter_direct(filter(faulty=true));

  Real Failure_fouling(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;

  // Failure definition
  filter.fouling = Failure_fouling;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Filter_faulty;
