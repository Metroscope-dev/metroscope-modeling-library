within MetroscopeModelingLibrary.FlueGases.HeatExchangers;
model AirAirHX
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.HeatExchangers.MonophasicHX(
    redeclare package Medium = FlueGasesMedium,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_cold_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_cold_out,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_hot_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_hot_out,
    redeclare MetroscopeModelingLibrary.FlueGases.Pipes.FrictionPipe hot_side_pipe,
    redeclare MetroscopeModelingLibrary.FlueGases.Pipes.FrictionPipe cold_side_pipe,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPFlowModel hot_side,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPFlowModel cold_side
    ) annotation();

  annotation (Icon(graphics={
        Polygon(
          points={{-160,80},{-160,0},{-160,-80},{-60,-80},{60,-80},{160,-80},{
              160,0},{160,80},{60,80},{-60,80},{-160,80}},
          lineColor={95,95,95},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-124,62},{-124,40},{-124,-40},{-124,-60},{-104,-60},{-12,-60},
              {94,-60},{116,-60},{116,-38},{116,43.0566},{116,62},{96,62},{-16,62},
              {-104,62},{-124,62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-130,50},{126,-42}},
          lineColor={95,95,95},
          lineThickness=1),
        Rectangle(
          extent={{-126,42},{124,-34}},
          lineColor={95,95,95},
          lineThickness=1),
        Rectangle(
          extent={{-130,34},{126,-26}},
          lineColor={95,95,95},
          lineThickness=1),
        Rectangle(
          extent={{116,52},{142,-54}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-148,54},{-124,-52}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          lineColor={0,140,72},
          radius=0)}));
end AirAirHX;
