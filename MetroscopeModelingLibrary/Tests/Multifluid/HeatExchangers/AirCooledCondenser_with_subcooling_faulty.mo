within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_with_subcooling_faulty
  extends AirCooledCondenser_with_subcooling_direct(airCooledCondenser_with_subcooling(faulty=true));

  Real Failure_fouling(start=0);
  Real Failure_air_intake(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10 * time;
  Failure_air_intake = 0 + 1e-3 * time;

  // Failure definition
  airCooledCondenser_with_subcooling.fouling = Failure_fouling;
  airCooledCondenser_with_subcooling.air_intake = Failure_air_intake;

  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end AirCooledCondenser_with_subcooling_faulty;
