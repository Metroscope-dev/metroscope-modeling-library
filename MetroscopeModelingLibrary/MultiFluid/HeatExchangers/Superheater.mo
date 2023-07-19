within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Superheater
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot",
                                                    Q_cold_0 = 100,
                                                    Q_hot_0 = 500,
                                                    T_cold_in_0 = 360 + 273.15,
                                                    T_cold_out_0 = 400 + 273.15,
                                                    T_hot_in_0 = 293 + 273.15,
                                                    T_hot_out_0 = 290.5 + 273.15,
                                                    P_cold_in_0 = 170e5,
                                                    P_cold_out_0 = 169e5,
                                                    P_hot_in_0 = 1.1e5,
                                                    P_hot_out_0 = 1.05e5,
                                                    h_cold_in_0 = 2.65e6,
                                                    h_cold_out_0 = 2.917e6,
                                                    h_hot_in_0 = 7e5,
                                                    h_hot_out_0 = 6.4e5)
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
