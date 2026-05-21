within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Flue_gas_stack
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                        annotation (IconMap(primitivesVisible=
          false));
  annotation (Icon(coordinateSystem(initialScale=0.5, extent={{-100,-20},{100,260}}), graphics={
        Rectangle(
          extent={{-40,250},{40,-20}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-10,52},{-10,56},{-8,62},{-14,64},{-10.0039,69.9961},{-10,76}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{8,52},{8,56},{10,62},{4,64},{7.9961,69.9961},{8,76}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{20,52},{20,56},{22,62},{16,64},{19.9961,69.9961},{20,76}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-20,52},{-20,56},{-18,62},{-24,64},{-20.0039,69.9961},{-20,76}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Rectangle(
          extent={{-40,20},{40,-20}},
          lineThickness=0.5,
          fillColor={48,48,48},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-20,264},{-20,268},{-18,274},{-24,276},{-20.004,281.996},{-20,288}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-10,264},{-10,268},{-8,274},{-14,276},{-10.004,281.996},{-10,288}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{8,264},{8,268},{10,274},{4,276},{7.996,281.996},{8,288}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{20,264},{20,268},{22,274},{16,276},{19.996,281.996},{20,288}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{-44,260},{44,240}},
          lineColor={0,0,127},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-42,226},{42,220}},
          lineColor={0,0,0},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Rectangle(
          extent={{-42,206},{42,200}},
          lineColor={0,0,0},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Rectangle(
          extent={{-42,186},{42,180}},
          lineColor={0,0,0},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}), Diagram(coordinateSystem(initialScale=0.5, extent={{-100,-20},{100,260}})));
end Flue_gas_stack;
