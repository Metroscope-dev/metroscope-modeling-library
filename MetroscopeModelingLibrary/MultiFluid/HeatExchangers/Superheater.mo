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
          extent={{-70,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{30,66},{30,-60},{10,-60},{10,64},{-10,64},{-10,-60},{-30,-60},{-30,66}},
          color={170,213,255},
          smooth=Smooth.Bezier,
          thickness=1),
        Line(
          points={{28,68},{30,-56},{10,-60},{12,66},{-12,66},{-10,-60},{-30,-56},{-28,68}},
          color={28,108,200},
          smooth=Smooth.Bezier),
        Line(
          points={{32,66},{32,-60},{8,-62},{10,62},{-10,62},{-8,-62},{-32,-60},{-32,66}},
          color={28,108,200},
          smooth=Smooth.Bezier)}));
end Superheater;
