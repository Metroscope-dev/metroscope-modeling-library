within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Superheater
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11)
 annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  annotation (Icon(graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid), Line(
          points={{40,80},{40,-72},{10,-72},{12,80},{-12,80},{-10,-72},{-40,-72},{-40,80}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1,
          pattern=LinePattern.Dash)}));
end Superheater;
