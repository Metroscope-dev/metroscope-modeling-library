within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model LiqLiqHX

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputArea S;
  Inputs.InputHeatExchangeCoefficient Kth;

  parameter String QCp_max_side = "cold";

  Units.Power W;
  Units.MassFlowRate Q_cold;
  Units.MassFlowRate Q_hot;
  Units.Temperature T_cold_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;


  Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-172,-10},{-152,10}}), iconTransformation(extent={{-172,-10},{-152,10}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(extent={{-10,70},{10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  Connectors.Outlet C_hot_out(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0)) annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));


  Pipes.WaterPipe cold_side_pipe(Q_0 = Q_cold_0)
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Pipes.WaterPipe hot_side_pipe(Q_0 = Q_hot_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,52})));
  BaseClasses.IsoPFlowModel hot_side(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={-1,21})));
  BaseClasses.IsoPFlowModel cold_side(Q_0=Q_cold_0) annotation (Placement(transformation(extent={{-26,-58},{22,-10}})));
  Power.HeatExchange.NTUHeatExchange HX(config="monophasic_counter_current", QCp_max_side = QCp_max_side) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-6})));
equation

  // Definitions
  Q_cold =cold_side.Q_in;
  Q_hot =hot_side.Q_in;
  T_cold_in =cold_side.T_in;
  T_cold_out =cold_side.T_out;
  T_hot_in =hot_side.T_in;
  T_hot_out =hot_side.T_out;
  cold_side.W = W;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Pressure losses
  cold_side_pipe.delta_z=0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z=0;
  hot_side_pipe.Kfr = Kfr_hot;

  // Power Exchange
  HX.W = W;
  HX.Kth =  Kth;
  HX.S = S;
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.Cp_cold =
    MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(
    cold_side.state_in);
  HX.Cp_hot =
    MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(
    hot_side.state_in);


  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(
      points={{-120,0},{-52,0},{-52,-34},{-26,-34}},
      color={28,108,200},
      thickness=1));
  connect(cold_side.C_out, C_cold_out) annotation (Line(
      points={{22,-34},{80,-34},{80,0},{160,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{-140,0},{-162,0}},
      color={28,108,200},
      thickness=1));
  connect(C_hot_in, hot_side_pipe.C_in) annotation (Line(
      points={{0,80},{0,71},{1.77636e-15,71},{1.77636e-15,62}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_pipe.C_out, hot_side.C_in) annotation (Line(
      points={{0,42},{0,38},{50,38},{50,21},{22,21}},
      color={238,46,47},
      thickness=1));
  connect(hot_side.C_out, C_hot_out) annotation (Line(
      points={{-24,21},{-30,21},{-30,20},{-40,20},{-40,-60},{0,-60},{0,-80}},
      color={238,46,47},
      thickness=1));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},
            {160,80}}),      graphics={
        Polygon(
          points={{-160,80},{-160,0},{-160,-80},{-60,-80},{60,-80},{160,-80},{160,
              0},{160,80},{60,80},{-60,80},{-160,80}},
          lineColor={28,108,200},
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
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-130,50},{126,-42}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{-126,42},{124,-34}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{-130,34},{126,-26}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{116,52},{142,-54}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-148,54},{-124,-52}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0)}),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},{160,80}})));
end LiqLiqHX;
