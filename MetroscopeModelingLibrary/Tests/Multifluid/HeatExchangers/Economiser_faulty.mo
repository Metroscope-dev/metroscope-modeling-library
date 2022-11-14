within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_faulty
  extends Economiser_direct(economiser(faulty=true));

  Real Failure_fouling(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;

  // Failure definition
  economiser.fouling = Failure_fouling;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Economiser_faulty;
