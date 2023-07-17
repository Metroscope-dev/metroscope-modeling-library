within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178)
 annotation(IconMap(primitivesVisible=false));

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
