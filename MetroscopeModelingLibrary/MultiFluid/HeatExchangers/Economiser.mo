within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
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

  annotation (Icon(graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid), Line(
          points={{40,80},{40,-80},{10,-80},{12,80},{-12,80},{-14,-80},{-40,-80},{-40,78}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1)}));
end Economiser;
