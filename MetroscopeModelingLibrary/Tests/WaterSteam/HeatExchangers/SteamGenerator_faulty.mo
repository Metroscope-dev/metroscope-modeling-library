within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model SteamGenerator_faulty
  extends SteamGenerator_direct(steam_generator(faulty=true));

  Real Fault_feedwater_bias(start=0);

equation

  // Failure input
  Fault_feedwater_bias = 5 * time;

  // Failure definition
  steam_generator.feed_water_flow_rate_measurement_bias = Fault_feedwater_bias;

end SteamGenerator_faulty;
