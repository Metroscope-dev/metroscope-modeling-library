within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model airCooledCondenser_faulty
  extends airCooledCondenser_direct(airCooledCondenser(faulty=true));

  Real Failure_fouling=30;
  Real Failure_air_intake=0;

equation

  airCooledCondenser.fouling = Failure_fouling*time;
  airCooledCondenser.air_intake = 0;

end airCooledCondenser_faulty;
