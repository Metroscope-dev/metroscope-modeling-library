within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Reheater
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;

  Units.Power W_tot;
  Inputs.InputArea S_tot;
  Inputs.InputReal level(min= 0, max=1, start = 0.3);

  // Deheating
  Units.Power W_deheating;

  // Condensation
  Units.Area S_cond;
  Units.HeatExchangeCoefficient Kth_cond;
  Units.Power W_condensing;

  // Subcooling
  Units.Area S_subc;
  Inputs.InputHeatExchangeCoefficient Kth_subc;
  Units.Power W_subcooling;

  Units.SpecificEnthalpy h_vap_sat(start=2e6);
  Units.SpecificEnthalpy h_liq_sat(start=1e5);
  Units.Temperature Tsat;

  parameter String HX_config_condensing="condenser_counter_current";
  parameter String HX_config_subcooling= "shell_and_tubes";
  parameter String QCp_max_side_subcooling = "cold";

  Units.InletMassFlowRate Q_cold(start=Q_cold_0, nominal=Q_cold_0);
  Units.InletMassFlowRate Q_hot(start=Q_hot_0, nominal=Q_hot_0);
  Units.Temperature T_cold_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-172,-10},{-152,10}}), iconTransformation(extent={{-172,-10},{-152,10}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(extent={{-10,70},{10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0)) annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));

  Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0) annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,56})));
  BaseClasses.IsoPFlowModel hot_side_deheating(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={101,19})));
  BaseClasses.IsoPFlowModel cold_side_deheating(Q_0=Q_cold_0) annotation (Placement(transformation(extent={{80,-58},{128,-10}})));
  BaseClasses.IsoPFlowModel hot_side_condensing(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={23,19})));
  BaseClasses.IsoPFlowModel cold_side_condensing(Q_0=Q_cold_0) annotation (Placement(transformation(extent={{0,-58},{48,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_condensing(config=HX_config_condensing) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={24,-6})));
  BaseClasses.IsoPFlowModel hot_side_subcooling(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={-55,19})));
  BaseClasses.IsoPFlowModel cold_side_subcooling(Q_0=Q_cold_0) annotation (Placement(transformation(extent={{-78,-58},{-30,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_subcooling(config=HX_config_subcooling, QCp_max_side = QCp_max_side_subcooling)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-54,-6})));
equation
  // Definitions
  Q_cold = cold_side_condensing.Q_in;
  Q_hot = hot_side_deheating.Q_in;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side_deheating.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side_subcooling.T_out;
  W_tot = W_deheating + W_condensing + W_subcooling;

  Tsat = hot_side_deheating.T_out;
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));

  // Pressure losses
  cold_side_pipe.delta_z = 0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z = 0;
  hot_side_pipe.Kfr = Kfr_hot;

  /* Deheating */
  // Energy balance
  hot_side_deheating.W + cold_side_deheating.W = 0;
  cold_side_deheating.W = W_deheating;

  // Power Exchange
  if hot_side_deheating.h_in > h_vap_sat then
      hot_side_deheating.h_out = h_vap_sat; // if steam is superheated, it is first deheated
  else
      hot_side_deheating.h_out = hot_side_deheating.h_in;
  end if;


  /* Water level */
  S_tot = S_cond + S_subc;  // Deheating surface is neglected
  S_subc = level * S_tot;

  /* Condensing */
  // Energy Balance
  hot_side_condensing.W + cold_side_condensing.W = 0;
  cold_side_condensing.W = W_condensing;

  // Power Exchange
  hot_side_condensing.h_out = h_liq_sat;

  HX_condensing.W = W_condensing;
  HX_condensing.Kth = Kth_cond;
  HX_condensing.S = S_cond;
  HX_condensing.Q_cold = Q_cold;
  HX_condensing.Q_hot = Q_hot;
  HX_condensing.T_cold_in = cold_side_condensing.T_in;
  HX_condensing.T_hot_in = hot_side_condensing.T_in;
  HX_condensing.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_condensing.state_in);
  HX_condensing.Cp_hot = WaterSteamMedium.specificHeatCapacityCp(hot_side_condensing.state_in);

  /* Subcooling */
  // Energy Balance
  hot_side_subcooling.W + cold_side_subcooling.W = 0;
  cold_side_subcooling.W = W_subcooling;

  // Power exchange
  HX_subcooling.W = W_subcooling;
  HX_subcooling.Kth = Kth_subc;
  HX_subcooling.S = S_subc;
  HX_subcooling.Q_cold = Q_cold;
  HX_subcooling.Q_hot = Q_hot;
  HX_subcooling.T_cold_in = cold_side_subcooling.T_in;
  HX_subcooling.T_hot_in = hot_side_subcooling.T_in;
  HX_subcooling.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_subcooling.state_in);
  HX_subcooling.Cp_hot = WaterSteamMedium.specificHeatCapacityCp(hot_side_subcooling.state_in);

  connect(cold_side_deheating.C_out, C_cold_out) annotation (Line(
      points={{128,-34},{144,-34},{144,0},{160,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{-140,0},{-162,0}},
      color={28,108,200},
      thickness=1));
  connect(C_hot_in, hot_side_pipe.C_in) annotation (Line(
      points={{0,80},{0,71},{1.77636e-15,71},{1.77636e-15,66}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_pipe.C_out, hot_side_deheating.C_in) annotation (Line(
      points={{-1.77636e-15,46},{-1.77636e-15,38},{132,38},{132,19},{124,19}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_deheating.C_out, hot_side_condensing.C_in) annotation (Line(
      points={{78,19},{46,19}},
      color={238,46,47},
      thickness=1));

  connect(cold_side_condensing.C_out, cold_side_deheating.C_in) annotation (
      Line(
      points={{48,-34},{80,-34}},
      color={28,108,200},
      thickness=1));

  connect(hot_side_condensing.C_out, hot_side_subcooling.C_in) annotation (Line(
      points={{-3.55271e-15,19},{-32,19}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_subcooling.C_out, C_hot_out) annotation (Line(
      points={{-78,19},{-78,18},{-102,18},{-102,-62},{0,-62},{0,-80}},
      color={238,46,47},
      thickness=1));
  connect(cold_side_condensing.C_in, cold_side_subcooling.C_out) annotation (
      Line(
      points={{0,-34},{-30,-34}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_subcooling.C_in, cold_side_pipe.C_out) annotation (Line(
      points={{-78,-34},{-114,-34},{-114,0},{-120,0}},
      color={28,108,200},
      thickness=1));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},
            {160,80}}),      graphics={
        Polygon(
          points={{-160,80},{-160,60},{-160,-62.5},{-160,-80},{-120,-80},{10,-80},
              {160,-80},{160,80},{10,80},{-120,80},{-160,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-98,60},{-98,38},{-98,-42},{-98,-62},{-78,-62},{14,-62},{142,
              -62},{142,60},{10,60},{-78,60},{-98,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-126,-18},{-138,-18},{-138,-48},{-136,-60},{-80,-62},{12,-62},
              {122,-62},{144,-18},{126,-18},{10,-20},{-112,-18},{-126,-18}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-108,42},{-116,44},{-116,-44},{-108,-44},{-98,-44},{16,-44},{
              124,-44},{126,42},{12,42},{-100,42},{-108,42}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-114,48},{-120,48},{-120,-50},{-112,-50},{-100,-50},{10,-50},
              {132,-50},{132,48},{10,48},{-100,48},{-114,48}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-114,30},{-120,30},{-120,-32},{-112,-32},{-100,-32},{14,-32},
              {112,-32},{110,28},{12,30},{-100,30},{-114,30}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-118,36},{-124,36},{-124,-36},{-116,-38},{-106,-38},{6,-38},{
              116,-40},{120,34},{8,36},{-104,36},{-118,36}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{-150,50},{-100,-66}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-148,60},{-102,24}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{-148,52},{-100,18}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-120,60},{-100,38}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{20,23},{-20,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10,
          origin={-123,-40},
          rotation=90),
        Rectangle(
          extent={{10.5,11.5},{-10.5,-11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={-111.5,-49.5},
          rotation=90),
        Rectangle(
          extent={{14,23},{-14,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={-123,-34},
          rotation=90)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},{160,80}})));
end Reheater;
