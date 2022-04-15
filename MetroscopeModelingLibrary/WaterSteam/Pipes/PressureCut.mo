within MetroscopeModelingLibrary.WaterSteam.Pipes;
model PressureCut
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Pipes.PressureCut(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-40,-60},{0,60}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{0,-60},{40,60}},
          color={0,0,0},
          thickness=1)}));
end PressureCut;
