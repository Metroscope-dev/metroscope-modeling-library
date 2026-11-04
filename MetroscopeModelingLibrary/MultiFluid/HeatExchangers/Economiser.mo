within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
  extends MetroscopeModelingLibrary.Utilities.Icons.HeatExchangePackage.MonophasicHXIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
    T_cold_out_0=521.744,
    T_hot_out_0=560.5332,
    P_cold_out_0=1.697654e+07,
    P_hot_out_0=110000,
    h_cold_in_0=979857,
    h_cold_out_0=1079652,
    h_hot_in_0=583163.5,
    h_hot_out_0=569909.5,
                                                    QCp_max_side = "unknown")
 annotation(IconMap(primitivesVisible=false));

 // Indicators
 Units.DifferentialTemperature FTR(start=T_cold_out_0-T_cold_in_0) "Feedwater Temperature Rise";

equation

  // Indicators
  FTR = T_cold_out - T_cold_in;

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, initialScale=0.5)), Icon(coordinateSystem(preserveAspectRatio=false, initialScale=0.5)));
end Economiser;
