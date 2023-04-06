within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_faulty
  extends AirCooledCondenser_direct(airCooledCondenser(faulty=true));

  Real Fault_fouling(start=0);
  Real Fault_air_intake(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10 * time;
  Fault_air_intake = 0 + 1e-3 * time;

  // Failure definition
  airCooledCondenser.fouling = Fault_fouling;
  airCooledCondenser.air_intake = Fault_air_intake;

  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end AirCooledCondenser_faulty;
