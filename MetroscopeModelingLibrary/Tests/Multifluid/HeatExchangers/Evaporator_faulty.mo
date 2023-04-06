within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_faulty
   extends Evaporator_direct(evaporator(faulty=true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;

  // Failure definition
  evaporator.fouling = Fault_fouling;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_faulty;
