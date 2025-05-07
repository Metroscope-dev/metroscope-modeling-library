within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Reheater
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;

  Units.Power W;
  Inputs.InputArea S;
  Inputs.InputFraction level(min= 0, max=1, start = 0.3);

  // Deheating
  Units.Power W_deheat;

  // Condensation
  Units.Area S_cond;
  Units.HeatExchangeCoefficient Kth_cond;
  Units.Power W_cond;

  // Subcooling
  Units.Area S_subc;
  Inputs.InputHeatExchangeCoefficient Kth_subc;
  Units.Power W_subc;

  Units.SpecificEnthalpy h_vap_sat(start=h_vap_sat_0);
  Units.SpecificEnthalpy h_liq_sat(start=h_liq_sat_0);
  Units.Temperature Tsat;

  parameter String HX_config_condensing="condenser";
  parameter String HX_config_subcooling="monophasic_cross_current"; // In subcooling zone, there is only the bottom part of the U-shaped tubes, so it is considered as cross current.
  parameter String QCp_max_side_subcooling = "cold";

  Units.PositiveMassFlowRate Q_cold_in(start=Q_cold_0, nominal=Q_cold_0);
  Units.PositiveMassFlowRate Q_hot_in(start=Q_hot_0, nominal=Q_hot_0);
  Units.NegativeMassFlowRate Q_cold_out(start=-Q_cold_0, nominal=-Q_cold_0); // Should be equal to Q_cold_in if no internal leaks
  Units.NegativeMassFlowRate Q_hot_out(start=-Q_hot_0, nominal=-Q_hot_0); // Should be equal to Q_hot_in if no internal leaks
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  // Indicators
  Units.DifferentialTemperature FTR(start=T_cold_out_0-T_cold_in_0) "Feedwater Temperature Rise";
  Units.DifferentialTemperature TTD(start=T_hot_in_0-T_cold_out_0) "Terminal Temperature Difference";
  Units.DifferentialTemperature DCA(start=T_hot_out_0-T_cold_in_0) "Drain Cooler Approach";
  Units.DifferentialTemperature pinch(start=min(T_hot_in_0-T_cold_out_0,T_hot_out_0-T_cold_in_0)) "Lowest temperature difference";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage
  Units.Fraction water_level_rise;  // Water level rise (can be negative)
  Units.MassFlowRate partition_plate_leak;  // Separating plate leak
  Units.MassFlowRate tube_rupture_leak; // Tube rupture leak : cold water leaks and mixes with the condensed steam

  // Initialization parameters
  parameter Units.PositiveMassFlowRate Q_cold_0=500;
  parameter Units.PositiveMassFlowRate Q_hot_0=50;
  parameter Units.Pressure P_cold_in_0 = 10e5;
  parameter Units.Pressure P_cold_out_0 = 9e5;
  parameter Units.Pressure P_hot_in_0 = 11e5;
  parameter Units.Pressure P_hot_out_0 = 11e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 50;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 100;
  parameter Units.Temperature T_hot_in_0 = 273.15 + 200;
  parameter Units.Temperature T_hot_out_0 = 273.15 + 150;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 2e5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 7e5;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 2.9e6;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 6e5;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0))
                                                annotation (Placement(transformation(extent={{-172,-10},{-152,10}}), iconTransformation(extent={{-172,-10},{-152,10}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0))
                                              annotation (Placement(transformation(extent={{-10,70},{10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0),
    P(start=P_hot_out_0),
    h_outflow(start=h_hot_out_0))                annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0),
    h_outflow(start=h_cold_out_0))                 annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));

  Pipes.FrictionPipe cold_side_pipe(
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,
    Q_0=Q_cold_0,
    T_0=T_cold_in_0,
    h_0=h_cold_in_0) annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Pipes.FrictionPipe hot_side_pipe(
    P_in_0=P_hot_in_0,
    P_out_0=P_hot_out_0,
    Q_0=Q_hot_0,
    T_0=T_hot_in_0,
    h_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,56})));
  BaseClasses.IsoPFlowModel hot_side_deheating(
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_in_0,
    h_in_0=h_hot_in_0,
    h_out_0=h_vap_sat_0,                       Q_0=Q_hot_0,
    P_0=P_hot_out_0)                                        annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={101,19})));
  BaseClasses.IsoPFlowModel cold_side_deheating(
    T_in_0=T_cold_out_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_out_0,
    h_out_0=h_cold_out_0,                       Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                         annotation (Placement(transformation(extent={{80,-58},{128,-10}})));
  BaseClasses.IsoPFlowModel hot_side_condensing(
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_in_0,
    h_in_0=h_vap_sat_0,
    h_out_0=h_liq_sat_0,                        Q_0=Q_hot_0,
    P_0=P_hot_out_0)                                         annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={23,19})));
  BaseClasses.IsoPFlowModel cold_side_condensing(
    T_in_0=0.5*(T_cold_in_0 + T_cold_out_0),
    T_out_0=T_cold_out_0,
    h_in_0=0.5*(h_cold_in_0 + h_cold_out_0),
    h_out_0=h_cold_out_0,                        Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                          annotation (Placement(transformation(extent={{0,-58},{48,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_condensing(config=HX_config_condensing, Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0,
                                                   T_hot_in_0=T_hot_in_0, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={24,-10})));
  BaseClasses.IsoPFlowModel hot_side_subcooling(
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_out_0,
    h_in_0=h_liq_sat_0,
    h_out_0=h_hot_out_0,                        Q_0=Q_hot_0,
    P_0=P_hot_out_0)                                         annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={-55,19})));
  BaseClasses.IsoPFlowModel cold_side_subcooling(
    T_in_0=T_cold_in_0,
    T_out_0=0.5*(T_cold_in_0 + T_cold_out_0),
    h_in_0=h_cold_in_0,
    h_out_0=0.5*(h_cold_in_0 + h_cold_out_0),    Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                          annotation (Placement(transformation(extent={{-78,-58},{-30,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_subcooling(config=HX_config_subcooling, QCp_max_side=QCp_max_side_subcooling, Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0,
                                                   T_hot_in_0=T_hot_in_0, T_cold_in_0=T_cold_in_0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-54,-10})));
  Pipes.Leak partition_plate annotation (Placement(transformation(extent={{-110,-78},{-90,-58}})));
  BaseClasses.IsoPHFlowModel final_mix_cold annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={144,-18})));
  Pipes.Leak tube_rupture annotation (Placement(transformation(extent={{-94,-24},{-74,-4}})));
  BaseClasses.IsoPHFlowModel final_mix_hot annotation (Placement(transformation(extent={{-58,-72},{-38,-52}})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));

equation

  // Failure modes
  if not faulty then
    fouling = 0;
    water_level_rise = 0;
    partition_plate_leak = 0;
    tube_rupture_leak = 0;
  end if;

  // Definitions
  Q_cold_in = C_cold_in.Q;
  Q_hot_in = C_hot_in.Q;
  Q_cold_out = C_cold_out.Q;
  Q_hot_out = C_hot_out.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = final_mix_cold.T_out;
                                // A IsoPHFlowModel is necessary to have a full thermodynamic state with temperature calculation at the cold outlet
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = final_mix_hot.T_out;
  W = W_deheat + W_cond + W_subc;

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
  cold_side_deheating.W =W_deheat;

  // Power Exchange
  if hot_side_deheating.h_in > h_vap_sat then
      hot_side_deheating.h_out = h_vap_sat; // if steam is superheated, it is first deheated
  else
      hot_side_deheating.h_out = hot_side_deheating.h_in;
  end if;

  /* Water level */
  S = S_cond + S_subc;      // Deheating surface is neglected
  S_subc =(level + water_level_rise)*S;

  /* Condensing */
  // Energy Balance
  hot_side_condensing.W + cold_side_condensing.W = 0;
  cold_side_condensing.W =W_cond;

  // Power Exchange
  hot_side_condensing.h_out = h_liq_sat;

  HX_condensing.W =W_cond;
  HX_condensing.Kth = Kth_cond * (1-fouling/100);
  HX_condensing.S = S_cond;
  HX_condensing.Q_cold = cold_side_deheating.Q;
  HX_condensing.Q_hot = hot_side_deheating.Q;
  HX_condensing.T_cold_in = cold_side_condensing.T_in;
  HX_condensing.T_hot_in = hot_side_condensing.T_in;
  HX_condensing.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_condensing.state_in);
  HX_condensing.Cp_hot = 0; // Not used by NTU method in condenser mode

  /* Subcooling */
  // Energy Balance
  hot_side_subcooling.W + cold_side_subcooling.W = 0;
  cold_side_subcooling.W =W_subc;

  // Power exchange
  HX_subcooling.W =W_subc;
  HX_subcooling.Kth = Kth_subc * (1-fouling/100);
  HX_subcooling.S = S_subc;
  HX_subcooling.Q_cold = cold_side_subcooling.Q;
  HX_subcooling.Q_hot = hot_side_subcooling.Q;
  HX_subcooling.T_cold_in = cold_side_subcooling.T_in;
  HX_subcooling.T_hot_in = hot_side_subcooling.T_in;
  HX_subcooling.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_subcooling.state_in);
  HX_subcooling.Cp_hot = WaterSteamMedium.specificHeatCapacityCp(hot_side_subcooling.state_in);

  // Indicators
  FTR = T_cold_out - T_cold_in;
  TTD = T_hot_in - T_cold_out;
  DCA = T_hot_out - T_cold_in;
  pinch = min(TTD, DCA);
  assert(pinch > 0, "A negative pinch is reached", AssertionLevel.warning); // Ensure a positive pinch
  assert(pinch > 1 or pinch < 0,  "A very low pinch (<1) is reached", AssertionLevel.warning); // Ensure a sufficient pinch

  // Internal leaks
  partition_plate.Q = 1e-5 + partition_plate_leak;
  tube_rupture.Q = 1e-5 + tube_rupture_leak;

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

  connect(partition_plate.C_in, cold_side_pipe.C_out) annotation (Line(points={{-110,-68},{-114,-68},{-114,0},{-120,0}}, color={217,67,180}));
  connect(hot_side_condensing.C_out, hot_side_subcooling.C_in) annotation (Line(
      points={{-3.55271e-15,19},{-32,19}},
      color={238,46,47},
      thickness=1));
  connect(cold_side_condensing.C_in, cold_side_subcooling.C_out) annotation (
      Line(
      points={{0,-34},{-30,-34}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_deheating.C_out, final_mix_cold.C_in) annotation (Line(
      points={{128,-34},{144,-34},{144,-28}},
      color={28,108,200},
      thickness=1));
  connect(final_mix_cold.C_out, C_cold_out) annotation (Line(
      points={{144,-8},{144,0},{160,0}},
      color={28,108,200},
      thickness=1));

  connect(partition_plate.C_out, final_mix_cold.C_in) annotation (Line(points={{-90,-68},{144,-68},{144,-28}}, color={217,67,180}));

  connect(tube_rupture.C_in, cold_side_pipe.C_out) annotation (Line(points={{-94,-14},{-114,-14},{-114,0},{-120,0}}, color={217,67,180}));
  connect(cold_side_subcooling.C_in, cold_side_pipe.C_out) annotation (Line(
      points={{-78,-34},{-114,-34},{-114,0},{-120,0}},
      color={28,108,200},
      thickness=1));
  connect(final_mix_hot.C_out, C_hot_out) annotation (Line(points={{-38,-62},{0,-62},{0,-80}}, color={238,46,47},
      thickness=1));
  connect(tube_rupture.C_out, final_mix_hot.C_in) annotation (Line(points={{-74,-14},{-66,-14},{-66,-6},{-102,-6},{-102,-62},{-58,-62}}, color={217,67,180}));
  connect(hot_side_subcooling.C_out, final_mix_hot.C_in) annotation (Line(
      points={{-78,19},{-78,18},{-102,18},{-102,-62},{-58,-62}},
      color={255,0,0},
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
          fillColor={255,170,170},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-126,-18},{-138,-18},{-138,-48},{-136,-60},{-80,-62},{12,-62},
              {122,-62},{144,-18},{126,-18},{10,-20},{-112,-18},{-126,-18}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-88,-8},{-84,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=360),
        Ellipse(
          extent={{-74,-4},{-70,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-54,-8},{-50,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,-8},{-26,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-10,-6},{-6,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,0},{14,-6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,-6},{40,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,-10},{74,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{114,14},{118,8}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,8},{-16,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{0,4},{4,-2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,4},{28,-2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,2},{60,-4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{86,22},{90,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{80,6},{84,0}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{110,-2},{114,-8}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-94,20},{-90,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{16,22},{20,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,20},{-56,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,20},{-32,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,22},{-12,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-82,10},{-78,4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-54,4},{-50,-2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,18},{54,12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-4,58.75},{4,58.75},{4,24.75},{8,24.75},{0,9.25},{-8,24.75},{-4,24.75},{-4,58.75}},
          lineThickness=1,
          fillColor={208,40,42},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
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
        Polygon(
          points={{-148,60},{-148,36},{-148,20},{-148,20},{-122,20},{-102,20},{-102,20},{-102,60},{-102,60},{-120,60},{-148,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-148,-62},{-148,-38},{-148,-22},{-148,-22},{-122,-22},{-102,-22},{-102,-22},{-102,-62},{-102,-62},{-120,-62},{-148,-62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0})}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},{160,80}}), graphics={
                                   Text(
          extent={{-6,10},{54,-2}},
          textColor={28,108,200},
          textString="Condensing"),
                  Text(
          extent={{72,10},{132,-2}},
          textColor={28,108,200},
          textString="Deheating"), Text(
          extent={{-84,10},{-24,-2}},
          textColor={28,108,200},
          textString="Subcooling")}));
end Reheater;
