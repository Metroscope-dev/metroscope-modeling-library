within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Superheater
  extends MetroscopeModelingLibrary.Utilities.Icons.HeatExchangePackage.MonophasicHXIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot",
                                                    T_cold_in_0 = 410 + 273.15,
                                                    T_cold_out_0 = 515 + 273.15,
                                                    T_hot_in_0 = 600 + 273.15,
                                                    T_hot_out_0 = 565 + 273.15,
                                                    P_cold_in_0 = 130e5,
                                                    P_cold_out_0 = 129.5e5,
                                                    P_hot_in_0 = 1.1e5,
                                                    P_hot_out_0 = 1.05e5,
                                                    h_cold_in_0 = 3.06e6,
                                                    h_cold_out_0 = 3.38e6,
                                                    h_hot_in_0 = 9.6e5,
                                                    h_hot_out_0 = 9.1e5)
 annotation(IconMap(primitivesVisible=false));

 // Indicators
 Units.DifferentialTemperature STR(start=T_cold_out_0-T_cold_in_0) "Steam Temperature Rise";
 Units.DifferentialTemperature DT_superheat(start=T_cold_out_0-WaterSteamMedium.saturationTemperature(P_cold_in_0)) "Superheat temperature difference";

equation

  // Indicators
  STR = T_cold_out - T_cold_in;
  DT_superheat = T_cold_out - WaterSteamMedium.saturationTemperature(cold_side_pipe.P_in);

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, initialScale=0.6)), Icon(coordinateSystem(preserveAspectRatio=false, initialScale=0.6)));
end Superheater;
