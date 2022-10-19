within MetroscopeModelingLibrary.Examples.CCGT;
model Evaporator_noRecirculation_direct
              // Boundary conditions
  input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
  input Units.MassFlowRate Q_hot_source(start=586);
  input Real hot_source_h(start=494000);

  input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
  input Units.MassFlowRate Q_cold_source(start=96);
  input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

   // Parameters
  parameter Units.Area S = 10;
  parameter Units.HeatExchangeCoefficient Kth= 102000;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;

  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
  WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
  WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
  FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
  FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
equation

    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source;
  cold_source.T_out =  T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  evaporator.S_vaporising = S;
  evaporator.Kth = Kth;
  evaporator.Kfr_hot = Kfr_hot;
  evaporator.Kfr_cold = Kfr_cold;

  connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
  connect(flashTank.C_steam_out, cold_steam_sink.C_in) annotation (Line(points={{-46,52},{-85,52}}, color={28,108,200}));
  connect(flashTank.C_liquid_out, cold_liquid_sink.C_in) annotation (Line(points={{-46,44},{-76,44},{-76,-52},{-83,-52}}, color={28,108,200}));
  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{28.3,-0.72},{45.65,-0.72},{45.65,0},{59,0}}, color={95,95,95}));
  connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,76},{-4,40}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
          lineColor={28,108,200},
          lineThickness=1,
          smooth=Smooth.Bezier,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,56},{-34,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-32,58},{-28,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-32,52},{-30,50}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
          Rectangle(
          extent={{-60,40},{78,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_noRecirculation_direct;
