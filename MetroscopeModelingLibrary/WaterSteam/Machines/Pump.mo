within MetroscopeModelingLibrary.WaterSteam.Machines;
model Pump
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Machines.Pump(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium)
                                                annotation(IconMap(primitivesVisible=false));

  annotation (
    Diagram(coordinateSystem(
        extent={{-100,-100},{100,100}}, initialScale=0.2),
                     graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}),
    Icon(coordinateSystem(
        extent={{-100,-100},{100,100}}, initialScale=0.2),
                     graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}}),
        Line(
          points={{70,90},{100,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{90,70},{100,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{-100,-100},{100,100}},
          color={0,0,0},
          thickness=0.5)}));
end Pump;
