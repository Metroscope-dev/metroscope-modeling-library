within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_faulty
  extends Economiser_direct(economiser(faulty=true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;

  // Failure definition
  economiser.fouling = Fault_fouling;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Economiser_faulty;
