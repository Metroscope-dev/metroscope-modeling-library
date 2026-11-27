within MetroscopeModelingLibrary.Tests.SteelworksGases.HeatExchangers;
model SteelworksGasesHeater_faulty "Steady operating point with 20% fouling"
  extends SteelworksGasesHeater_direct(heater(faulty=true));
equation
  heater.fouling = 20;
end SteelworksGasesHeater_faulty;
