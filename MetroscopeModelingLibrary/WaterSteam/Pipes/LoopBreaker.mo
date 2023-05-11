within MetroscopeModelingLibrary.WaterSteam.Pipes;
model LoopBreaker
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Pipes.LoopBreaker(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
        Ellipse(
          extent={{-36,40},{44,-40}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Line(
          points={{44,0},{96,0}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-36,0},{-94,0},{-92,0}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-20,-4},{-20,10},{20,10}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{16,14},{20,10},{16,6}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-20,-7},{-20,7},{20,7}},
          color={28,108,200},
          thickness=1,
          origin={6,-5},
          rotation=180),
        Line(
          points={{-2,4},{2,0},{-2,-4}},
          color={28,108,200},
          thickness=1,
          origin={-12,-12},
          rotation=180)}));
end LoopBreaker;
