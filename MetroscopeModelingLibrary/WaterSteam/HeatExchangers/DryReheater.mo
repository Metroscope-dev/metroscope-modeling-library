within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model DryReheater
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputArea S_condensing;
  Units.HeatExchangeCoefficient Kth;

  Units.SpecificEnthalpy h_vap_sat(start=h_vap_sat_0);
  Units.SpecificEnthalpy h_liq_sat(start=h_liq_sat_0);
  Units.Temperature Tsat;

  Units.Power W_deheating;
  Units.Power W_condensing;
  parameter String HX_config="condenser";

  Units.PositiveMassFlowRate Q_cold(
    start=Q_cold_0,
    nominal=Q_cold_0,
    min=1e-5);
  Units.PositiveMassFlowRate Q_hot(start=Q_hot_0, nominal=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Pressure P_cold_in_0 = 10e5;
  parameter Units.Pressure P_cold_out_0 = 9e5;
  parameter Units.Pressure P_hot_in_0 = 20e5;
  parameter Units.Pressure P_hot_out_0 = 20e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 50;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 100;
  parameter Units.Temperature T_hot_in_0 = 273.15 + 220;
  parameter Units.Temperature T_hot_out_0 = 273.15 + 220;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 5e5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 7e5;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 2.8e6;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 6e5;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0))
                                                annotation (Placement(transformation(extent={{-172,-10},{-152,10}}), iconTransformation(extent={{-172,-10},{-152,10}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0))
                                              annotation (Placement(transformation(extent={{-10,70},{10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0),
    P(start=P_cold_out_0),
    h_outflow(start=h_cold_out_0))                 annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));

  Pipes.Pipe cold_side_pipe(
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,   Q_0=Q_cold_0,
    T_0=T_cold_in_0,
    h_0=h_cold_in_0)                      annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Pipes.Pipe hot_side_pipe(
    P_in_0=P_hot_in_0,
    P_out_0=P_hot_out_0,   Q_0=Q_hot_0,
    T_0=T_hot_in_0,
    h_0=h_hot_in_0)                     annotation (Placement(transformation(
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
        origin={71,19})));
  BaseClasses.IsoPFlowModel cold_side_deheating(
    T_in_0=T_cold_out_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_out_0,
    h_out_0=h_cold_out_0,                       Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                         annotation (Placement(transformation(extent={{48,-58},{96,-10}})));
  BaseClasses.IsoPFlowModel hot_side_condensing(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={-59,21})));
  BaseClasses.IsoPFlowModel cold_side_condensing(
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_out_0,                        Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                          annotation (Placement(transformation(extent={{-82,-58},{-34,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_condensing(config=HX_config, Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0,
                                                   T_hot_in_0=T_hot_in_0, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-60,0})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  // Definitions
  Q_cold = cold_side_condensing.Q_in;
  Q_hot = hot_side_deheating.Q_in;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side_deheating.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side_condensing.T_out;
  Tsat = hot_side_deheating.T_out;

  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));


  // Pressure losses
  cold_side_pipe.delta_z=0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z=0;
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


  /* Condensing */
  // Energy Balance
  hot_side_condensing.W + cold_side_condensing.W = 0;
  cold_side_condensing.W = W_condensing;

  // Power Exchange
  hot_side_condensing.h_out = h_liq_sat;

  HX_condensing.W = W_condensing;
  HX_condensing.Kth = Kth*(1-fouling/100);
  HX_condensing.S = S_condensing;
  HX_condensing.Q_cold = Q_cold;
  HX_condensing.Q_hot = Q_hot;
  HX_condensing.T_cold_in = cold_side_condensing.T_in;
  HX_condensing.T_hot_in = Tsat;
  HX_condensing.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_condensing.state_in);
  HX_condensing.Cp_hot = 0; // Not used by NTU method in condenser mode

  connect(cold_side_deheating.C_out, C_cold_out) annotation (Line(
      points={{96,-34},{144,-34},{144,0},{160,0}},
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
      points={{-1.77636e-15,46},{-1.77636e-15,40},{134,40},{134,19},{94,19}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_deheating.C_out, hot_side_condensing.C_in) annotation (Line(
      points={{48,19},{48,20},{-36,20},{-36,21}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_condensing.C_out, C_hot_out) annotation (Line(
      points={{-82,21},{-84,21},{-84,22},{-104,22},{-104,-66},{0,-66},{0,-80}},
      color={238,46,47},
      thickness=1));

  connect(cold_side_condensing.C_in, cold_side_pipe.C_out) annotation (Line(
      points={{-82,-34},{-114,-34},{-114,0},{-120,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_condensing.C_out, cold_side_deheating.C_in) annotation (
      Line(
      points={{-34,-34},{48,-34}},
      color={28,108,200},
      thickness=1));

  annotation (Icon(coordinateSystem(extent={{-160,-80},{160,80}}),
                   graphics={
        Polygon(
          points={{-160,80},{-160,60},{-160,-62.5},{-160,-80},{-120,-80},{10,-80},
              {160,-80},{160,80},{10,80},{-120,80},{-160,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-98,58},{-98,36},{-98,-44},{-98,-64},{-78,-64},{14,-64},{142,
              -64},{142,58},{10,58},{-78,58},{-98,58}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-108,40},{-116,42},{-116,-46},{-108,-46},{-98,-46},{16,-46},{
              124,-46},{126,40},{12,40},{-100,40},{-108,40}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-114,46},{-120,46},{-120,-52},{-112,-52},{-100,-52},{10,-52},
              {132,-52},{132,46},{10,46},{-100,46},{-114,46}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-114,28},{-120,28},{-120,-34},{-112,-34},{-100,-34},{14,-34},
              {112,-34},{110,26},{12,28},{-100,28},{-114,28}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-118,34},{-124,34},{-124,-38},{-116,-40},{-106,-40},{6,-40},{
              116,-42},{120,32},{8,34},{-104,34},{-118,34}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{-148,50},{-98,-56}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-148,58},{-102,22}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{-148,50},{-100,16}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0),
        Rectangle(
          extent={{-120,58},{-100,36}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0),
        Rectangle(
          extent={{20,23},{-20,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10,
          origin={-123,-42},
          rotation=90),
        Rectangle(
          extent={{10.5,11.5},{-10.5,-11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0,
          origin={-111.5,-51.5},
          rotation=90),
        Rectangle(
          extent={{14,23},{-14,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0,
          origin={-123,-36},
          rotation=90)}), Diagram(coordinateSystem(extent={{-160,-80},{160,80}}),
        graphics={Text(
          extent={{40,42},{100,30}},
          textColor={28,108,200},
          textString="Deheating"), Text(
          extent={{-90,42},{-30,30}},
          textColor={28,108,200},
          textString="Condensing")}));
end DryReheater;
