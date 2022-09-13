within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Superheater_faulty
    extends Superheater_direct(superheater(faulty=true));

  Real Failure_fouling(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;

  // Failure definition
  superheater.fouling = Failure_fouling;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Superheater_faulty;
