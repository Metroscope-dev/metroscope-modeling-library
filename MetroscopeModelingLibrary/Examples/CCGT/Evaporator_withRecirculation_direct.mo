within MetroscopeModelingLibrary.Examples.CCGT;
model Evaporator_withRecirculation_direct
              // Boundary conditions
  input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
  input Utilities.Units.MassFlowRate Q_hot_source(start=586);
  input Real hot_source_h(start=494000);

  input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
  input Utilities.Units.MassFlowRate Q_cold_source(start=96);
  input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

   // Parameters
  parameter Utilities.Units.Area S=10;
  parameter Utilities.Units.HeatExchangeCoefficient Kth=102000;
  parameter Utilities.Units.FrictionCoefficient Kfr_hot=0;
  parameter Utilities.Units.FrictionCoefficient Kfr_cold=1;

  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
  WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-18,46},
            {16,80}})));
  WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{54,38},
            {74,58}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{54,70},
            {74,90}})));
  FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-10,-10},
            {10,10}},
        rotation=270,
        origin={-28,88})));
  FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-46,44})));
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

  // Recirculation flow
  evaporator.Q_cold = Q_cold_source;

  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{28.3,-0.72},{45.65,-0.72},{45.65,0},{59,0}}, color={95,95,95}));
  connect(cold_source.C_out, flashTank.C_in) annotation (Line(points={{-28,83},{
          -28,69.8},{-18,69.8}},                   color={28,108,200}));
  connect(flashTank.C_hot_steam, cold_steam_sink.C_in) annotation (Line(points={
          {16,69.8},{16,68},{48,68},{48,80},{59,80}}, color={28,108,200}));
  connect(cold_liquid_sink.C_in, flashTank.C_hot_liquid)
    annotation (Line(points={{59,48},{16,48},{16,56.2}}, color={28,108,200}));
  connect(flashTank.C_hot_liquid, evaporator.C_cold_in) annotation (Line(points=
         {{16,56.2},{16,25.2},{12.7,25.2}}, color={28,108,200}));
  connect(evaporator.C_cold_out, pressureCut.C_in) annotation (Line(points={{-10.7,
          25.2},{-46,25.2},{-46,34}}, color={28,108,200}));
  connect(pressureCut.C_out, flashTank.C_in) annotation (Line(points={{-46,54},{
          -48,54},{-48,69.8},{-18,69.8}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-18,76},{18,40}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-18,54},{-8,42},{8,42},{14,50},{16,56},{16,58},{-12,60},{-16,58},{-16,54},{-18,54}},
          lineColor={28,108,200},
          lineThickness=1,
          smooth=Smooth.Bezier,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-14,56},{-12,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-10,58},{-6,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-10,52},{-8,50}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
          Rectangle(
          extent={{-60,40},{80,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-14,50},{-28,30},{-28,-16},{-6,-46},{8,-46},{20,-16},{18,20},{4,46}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{-10,50},{-24,30},{-24,-16},{-2,-46},{12,-46},{24,-16},{22,20},{8,46}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{-14,54},{-32,32},{-32,-14},{-10,-44},{4,-44},{16,-14},{14,22},{0,48}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_withRecirculation_direct;
