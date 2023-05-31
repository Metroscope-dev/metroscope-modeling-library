within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Economiser
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  extends Partial.HeatExchangers.WaterFlueGasesMonophasicHX(
                                                    QCp_max_side = "hot")
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
