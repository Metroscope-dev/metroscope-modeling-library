within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
  extends MetroscopeModelingLibrary.Utilities.Icons.HeatExchangePackage.MonophasicHXIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot")
 annotation(IconMap(primitivesVisible=false));

 // Indicators
 Units.DifferentialTemperature FTR(start=T_cold_out_0-T_cold_in_0) "Feedwater Temperature Rise";

equation

  // Indicators
  FTR = T_cold_out - T_cold_in;

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, initialScale=0.6)), Icon(coordinateSystem(preserveAspectRatio=false, initialScale=0.6)));
end Economiser;
