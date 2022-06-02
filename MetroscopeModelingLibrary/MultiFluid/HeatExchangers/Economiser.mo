within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "cold",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178)
 annotation(IconMap(primitivesVisible=false));

  annotation (Icon(graphics={
          Rectangle(
          extent={{-70,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid), Line(
          points={{30,66},{30,-60},{10,-60},{10,64},{-10,64},{-10,-60},{-30,-60},{-30,66}},
          color={28,108,200},
          smooth=Smooth.Bezier,
          thickness=1)}));
end Economiser;
