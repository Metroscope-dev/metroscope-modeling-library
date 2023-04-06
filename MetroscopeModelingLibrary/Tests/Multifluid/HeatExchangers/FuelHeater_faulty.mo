within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model FuelHeater_faulty
  extends FuelHeater_direct(fuelHeater(faulty=true));

  Real Fault_fouling(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10*time;

  // Failure definition
  fuelHeater.fouling = Fault_fouling;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater_faulty;
