within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_with_subcooling_faulty
  extends AirCooledCondenser_with_subcooling_direct(airCooledCondenser_with_subcooling(faulty=true));

  Real Fault_fouling(start=0);
  Real Fault_air_intake(start=0);

equation

  // Failure input
  Fault_fouling = 0 + 10 * time;
  Fault_air_intake = 0 + 1e-3 * time;

  // Failure definition
  airCooledCondenser_with_subcooling.fouling = Fault_fouling;
  airCooledCondenser_with_subcooling.air_intake = Fault_air_intake;

  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end AirCooledCondenser_with_subcooling_faulty;
