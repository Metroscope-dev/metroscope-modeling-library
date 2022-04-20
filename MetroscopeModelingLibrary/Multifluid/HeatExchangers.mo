within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model Economiser
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "cold",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178);

<<<<<<< HEAD
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid), Line(
            points={{30,70},{30,-76},{0,-76},{0,72},{-30,72},{-30,-72}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end Economiser;

  model Superheater
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11);
    import MetroscopeModelingLibrary.Units.Inputs;

  public
    FlueGases.Connectors.Inlet C_hot_in annotation (Placement(transformation(
            extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},
              {-60,10}})));
    FlueGases.Connectors.Outlet C_hot_out annotation (Placement(transformation(
            extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,
              10}})));
    WaterSteam.Connectors.Inlet C_cold_in annotation (Placement(transformation(
            extent={{20,60},{40,80}}),   iconTransformation(extent={{20,60},{40,
              80}})));
    WaterSteam.Connectors.Outlet C_cold_out annotation (Placement(transformation(
            extent={{-40,-80},{-20,-60}}),
                                         iconTransformation(extent={{-40,-80},{
              -20,-60}})));
    FlueGases.Pipes.Pipe hot_side_pipe annotation (Placement(transformation(extent={{-48,18},{-28,38}})));
    Power.HeatExchange.NTUHeatExchange HX(config="monophasic_cross_current",QCp_max_side=QCp_max_side) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90)));
    FlueGases.BaseClasses.IsoPFlowModel hot_side annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,0})));
    WaterSteam.BaseClasses.IsoPFlowModel cold_side annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={22,2})));
    WaterSteam.Pipes.Pipe cold_side_pipe annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={22,-36})));
  equation
    connect(cold_side_pipe.C_out,cold_side. C_in)
      annotation (Line(points={{40,-16},{40,2}},  color={28,108,200}));
    connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{40,-36},{
            40,-54},{40,70},{30,70}},color={28,108,200}));
    connect(cold_side.C_out, C_cold_out)
      annotation (Line(points={{40,22},{40,-70},{-30,-70}},
                                                          color={28,108,200}));
    connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-2,-2},{-2,-4},{70,
            -4},{70,0}}, color={95,95,95}));
    connect(C_hot_in, hot_side_pipe.C_in)
      annotation (Line(points={{-70,0},{-70,28},{-48,28}}, color={95,95,95}));
    connect(hot_side_pipe.C_out, hot_side.C_in)
      annotation (Line(points={{-28,28},{-2,28},{-2,18}}, color={95,95,95}));
=======
>>>>>>> 43b3d09 (ajout d'un superheater)
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid), Line(
            points={{30,70},{30,-76},{0,-76},{0,72},{-30,72},{-30,-72}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),          Line(
            points={{32,-72},{32,72},{-2,74},{0,-74},{-30,-74},{-28,72}},
            color={28,108,200},
            smooth=Smooth.Bezier),          Line(
            points={{28,-72},{30,70},{0,70},{2,-78},{-32,-76},{-32,70}},
            color={28,108,200},
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end Economiser;

  model Superheater
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11);
    import MetroscopeModelingLibrary.Units.Inputs;

  public
    FlueGases.Connectors.Inlet C_hot_in annotation (Placement(transformation(
            extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},
              {-60,10}})));
    FlueGases.Connectors.Outlet C_hot_out annotation (Placement(transformation(
            extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,
              10}})));
    WaterSteam.Connectors.Inlet C_cold_in annotation (Placement(transformation(
            extent={{20,-80},{40,-60}}), iconTransformation(extent={{20,-80},{40,-60}})));
    WaterSteam.Connectors.Outlet C_cold_out annotation (Placement(transformation(
            extent={{-40,60},{-20,80}}), iconTransformation(extent={{-40,60},{-20,80}})));
    FlueGases.Pipes.Pipe hot_side_pipe annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
    Power.HeatExchange.NTUHeatExchange HX(config="monophasic_cross_current", QCp_max_side=QCp_max_side)
                                                                                                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90)));
    FlueGases.BaseClasses.IsoPFlowModel hot_side annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,0})));
    WaterSteam.BaseClasses.IsoPFlowModel cold_side annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={22,2})));
    WaterSteam.Pipes.Pipe cold_side_pipe annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={22,-36})));
  equation
    connect(cold_side_pipe.C_out,cold_side. C_in)
      annotation (Line(points={{22,-26},{22,-8}}, color={28,108,200}));
    connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{22,-46},{22,-70},{30,-70}},
                                     color={28,108,200}));
    connect(cold_side.C_out, C_cold_out)
      annotation (Line(points={{22,12},{22,70},{-30,70}}, color={28,108,200}));
    connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-20,-10},{-20,-14},{70,-14},{70,0}},
                         color={95,95,95}));
    connect(hot_side_pipe.C_out,hot_side. C_in)
      annotation (Line(points={{-36,0},{-28,0},{-28,10},{-20,10}},
                                                          color={95,95,95}));
    connect(C_hot_in, hot_side_pipe.C_in) annotation (Line(points={{-70,0},{-56,0}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid), Line(
            points={{30,-74},{30,72},{0,72},{0,-76},{-30,-76},{-30,68}},
            color={205,225,255},
            thickness=1,
            smooth=Smooth.Bezier),          Line(
            points={{32,-72},{32,72},{-2,74},{0,-74},{-30,-74},{-28,72}},
            color={28,108,200},
            smooth=Smooth.Bezier),          Line(
            points={{28,-72},{30,70},{0,70},{2,-78},{-32,-76},{-32,70}},
            color={28,108,200},
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end Superheater;
end HeatExchangers;
