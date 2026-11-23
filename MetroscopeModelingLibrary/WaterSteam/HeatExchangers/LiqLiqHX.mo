within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model LiqLiqHX
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.HeatExchangers.LiqLiqHX(
    redeclare package Medium = WaterSteamMedium,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_cold_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_cold_out,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_hot_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_hot_out,
    redeclare MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe hot_side_pipe,
    redeclare MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe cold_side_pipe,
    redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPFlowModel hot_side,
    redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPFlowModel cold_side
    ) annotation();

  annotation (Icon(graphics={
        Polygon(
          points={{-160,80},{-160,0},{-160,-80},{-60,-80},{60,-80},{160,-80},{
              160,0},{160,80},{60,80},{-60,80},{-160,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-124,62},{-124,40},{-124,-40},{-124,-60},{-104,-60},{-12,-60},
              {94,-60},{116,-60},{116,-38},{116,43.0566},{116,62},{96,62},{-16,
              62},{-104,62},{-124,62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-130,50},{126,-42}},
          lineColor={0,0,255},
          lineThickness=1),
        Rectangle(
          extent={{-126,42},{124,-34}},
          lineColor={0,0,255},
          lineThickness=1),
        Rectangle(
          extent={{-130,34},{126,-26}},
          lineColor={0,0,255},
          lineThickness=1),
        Rectangle(
          extent={{116,52},{142,-54}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-148,54},{-124,-52}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,140,72},
          radius=0)}));
end LiqLiqHX;
