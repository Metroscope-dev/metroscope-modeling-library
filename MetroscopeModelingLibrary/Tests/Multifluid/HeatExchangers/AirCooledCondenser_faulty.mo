within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_faulty
  extends AirCooledCondenser_direct(airCooledCondenser(subcooling=true, faulty=true));

  Real Failure_fouling(start=0);
  Real Failure_air_intake(start=0);

equation

  // Failure input
  Failure_fouling = 0 + 10*time;
  Failure_air_intake = 0 + 1e-3 * time;

  // Failure definition
  airCooledCondenser.fouling = Failure_fouling;
  airCooledCondenser.air_intake = Failure_air_intake;

  annotation (Diagram(coordinateSystem(extent={{-100,-100},{100,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end AirCooledCondenser_faulty;
