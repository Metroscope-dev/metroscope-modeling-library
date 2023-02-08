within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Superheater
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11)
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
