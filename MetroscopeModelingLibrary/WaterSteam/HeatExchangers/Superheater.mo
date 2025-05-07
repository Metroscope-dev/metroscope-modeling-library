within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Superheater
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;

  // Deheating
  Units.Power W_deheat;

  // Condensation
  Units.Power W_cond;
  parameter String HX_config="condenser";
  Inputs.InputArea S;
  Units.HeatExchangeCoefficient Kth;
  Units.SpecificEnthalpy h_vap_sat_hot(start=h_vap_sat_0);
  Units.SpecificEnthalpy h_liq_sat_hot(start=h_liq_sat_0);
  Units.Temperature Tsat_hot;

  // Vaporising
  Units.Power W_vap;
  Units.SpecificEnthalpy h_vap_sat_cold(start=h_cold_in_0);
  Units.Temperature Tsat_cold(start=T_cold_in_0);

  // Ventilation
  Units.PositiveMassFlowRate Q_vent(start=Q_vent_0);
  Units.PositiveMassFlowRate Q_vent_faulty(start=Q_vent_0);

  // Definitions
  Units.PositiveMassFlowRate Q_cold_in(
    start=Q_cold_0,
    nominal=Q_cold_0,
    min=1e-5);
  Units.PositiveMassFlowRate Q_hot_in(start=Q_hot_0, nominal=Q_hot_0);
  Units.NegativeMassFlowRate Q_cold_out(
    start=-Q_cold_0,
    nominal=-Q_cold_0,
    max=1e-5);
  Units.NegativeMassFlowRate Q_hot_out(start=-Q_hot_0, nominal=-Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);
  Units.Power W;

  // Indicators
  Units.DifferentialTemperature DT_superheat(start=T_cold_out_0-T_cold_in_0) "Superheat temperature difference";
  Units.DifferentialTemperature TTD(start=T_hot_in_0-T_cold_out_0) "Terminal Temperature Difference";
  Units.DifferentialTemperature DCA(start=T_hot_out_0-T_cold_in_0) "Drain Cooler Approach";
  Units.DifferentialTemperature pinch(start=min(T_hot_in_0-T_cold_out_0,T_hot_out_0-T_cold_in_0)) "Lowest temperature difference";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage closed_vent(min = 0, max= 100); // Vent closing percentage
  Units.MassFlowRate tube_rupture_leak; // Tube rupture leak mass flow rate

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 1300;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.MassFlowRate Q_vent_0 = 1;
  parameter Units.Pressure P_cold_in_0 = 10e5;
  parameter Units.Pressure P_cold_out_0 = 10e5;
  parameter Units.Pressure P_hot_in_0 = 60e5;
  parameter Units.Pressure P_hot_out_0 = 59e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 200;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 250;
  parameter Units.Temperature T_hot_in_0 = 273.15 + 220;
  parameter Units.Temperature T_hot_out_0 = 273.15 + 220;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 2.6e6;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 2.8e6;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 2.9e6;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 1.2e6;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0))
                                                annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0))
                                              annotation (Placement(transformation(extent={{-170,-10},{-150,10}}),iconTransformation(extent={{-170,-10},{-150,10}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0),
    P(start=P_hot_out_0),
    h_outflow(start=h_hot_out_0))                annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0),
    P(start=P_cold_out_0),
    h_outflow(start=h_cold_out_0))                 annotation (Placement(transformation(extent={{-10,70},{10,90}}),iconTransformation(extent={{-10,70},{10,90}})));

  Pipes.FrictionPipe cold_side_pipe(
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,
    Q_0=Q_cold_0,
    T_0=T_cold_in_0,
    h_0=h_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={16,-64})));
  Pipes.FrictionPipe hot_side_pipe(
    P_in_0=P_hot_in_0,
    P_out_0=P_hot_out_0,
    Q_0=Q_hot_0,
    T_0=T_hot_in_0,
    h_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-130,0})));
  BaseClasses.IsoPFlowModel hot_side_deheating(
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_out_0,
    h_in_0=h_hot_in_0,
    h_out_0=h_vap_sat_0,                       Q_0=Q_hot_0,
    P_0=P_hot_out_0)                                        annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={-34,44})));
  BaseClasses.IsoPFlowModel cold_side_deheating(
    T_in_0=T_cold_out_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_out_0,
    h_out_0=h_cold_out_0,                       Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                         annotation (Placement(transformation(
        extent={{-15,-15},{15,15}},
        rotation=90,
        origin={39,45})));
  BaseClasses.IsoPFlowModel hot_side_condensing(
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_in_0,
    h_in_0=h_vap_sat_0,
    h_out_0=h_liq_sat_0,                        Q_0=Q_hot_0,
    P_0=P_hot_out_0)                                         annotation (Placement(transformation(
        extent={{-17,-16},{17,16}},
        rotation=270,
        origin={-34,1})));
  BaseClasses.IsoPFlowModel cold_side_condensing(
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_out_0,                        Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                          annotation (Placement(transformation(
        extent={{-17,-17},{17,17}},
        rotation=90,
        origin={39,1})));
  Power.HeatExchange.NTUHeatExchange HX_condensing(config=HX_config, Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0,
                                                   T_hot_in_0=T_hot_in_0, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-21},{10,21}},
        rotation=270,
        origin={3,2})));
  BaseClasses.IsoPFlowModel cold_side_vaporising(
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_in_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_in_0,                         Q_0=Q_cold_0,
    P_0=P_cold_out_0)                                          annotation (Placement(transformation(
        extent={{-17,-17},{17,17}},
        rotation=90,
        origin={39,-43})));
  BaseClasses.IsoPFlowModel hot_side_vaporising(
    T_in_0=T_hot_out_0,
    T_out_0=T_hot_out_0,
    h_in_0=h_liq_sat_0,
    h_out_0=h_liq_sat_0,                        Q_0=Q_cold_0,
    P_0=P_hot_out_0)                                          annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={-34,-44})));
  Connectors.Outlet C_vent(Q(start=-Q_vent_0), P(start=P_hot_out_0),
    h_outflow(start=h_vap_sat_0))
                           annotation (Placement(transformation(extent={{150,-88},{170,-68}})));

  Pipes.Leak tube_rupture annotation (Placement(transformation(extent={{-6,28},{14,48}})));
  BaseClasses.IsoPHFlowModel final_mix_cold annotation (Placement(transformation(extent={{24,52},{4,72}})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_hot_out_0));

equation

  // Failure modes
  if not faulty then
    closed_vent = 0;
    tube_rupture_leak = 0;
  end if;

  // Definitions
  Q_cold_in = C_cold_in.Q;
  Q_hot_in = C_hot_in.Q;
  Q_cold_out = C_cold_out.Q;
  Q_hot_out = C_hot_out.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = final_mix_cold.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side_vaporising.T_out;
  W = W_deheat + W_cond + W_vap;

  // Ventilation
  Q_vent_faulty = - C_vent.Q; // 1e-3 is used as a protection against zero flow in case the vent is totally closed
  Q_vent_faulty = Q_vent*(1-closed_vent/100) + 1e-3;
  // Pressure losses
  cold_side_pipe.delta_z = 0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z = 0;
  hot_side_pipe.Kfr = Kfr_hot;

  // Saturation
  Tsat_hot = hot_side_deheating.T_out;
  h_vap_sat_hot = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));
  h_liq_sat_hot = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(hot_side_deheating.P_in));
  Tsat_cold = cold_side_pipe.T_out;
  h_vap_sat_cold = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(cold_side_pipe.P_out));

  /* Deheating on hot side, superheating on cold side */

  // Energy balance
  hot_side_deheating.W + cold_side_deheating.W = 0;
  cold_side_deheating.W =W_deheat;

  // Power Exchange
  if hot_side_deheating.h_in > h_vap_sat_hot then
      hot_side_deheating.h_out = h_vap_sat_hot; // if steam is superheated, it is first deheated
  else
      hot_side_deheating.h_out = hot_side_deheating.h_in;
  end if;

  /* Condensing on hot side, superheating on cold side */

  // Energy Balance
  hot_side_condensing.W + cold_side_condensing.W = 0;
  cold_side_condensing.W =W_cond;

  // Power Exchange
  HX_condensing.W = W_cond;
  HX_condensing.Kth = Kth;
  HX_condensing.S = S;
  HX_condensing.Q_cold = cold_side_condensing.Q;
  HX_condensing.Q_hot = hot_side_condensing.Q;
  HX_condensing.T_cold_in = Tsat_cold;
  HX_condensing.T_hot_in = Tsat_hot;
  HX_condensing.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_condensing.state_in);
  HX_condensing.Cp_hot = 0; // Not used by NTU method in condenser mode

  /* Vaporising on cold side, condensation on hot side*/

  hot_side_vaporising.W + cold_side_vaporising.W = 0;
  W_vap = cold_side_vaporising.W;

  hot_side_vaporising.h_out = h_liq_sat_hot; // Hot steam is completely condensed

  if cold_side_vaporising.h_in < h_vap_sat_cold then
      cold_side_vaporising.h_out = h_vap_sat_cold;
  else
      cold_side_vaporising.h_out = cold_side_vaporising.h_in;
  end if;

  // Indicators
  DT_superheat = T_cold_out - Tsat_cold;
  TTD = T_hot_in - T_cold_out;
  DCA = T_hot_out - T_cold_in;
  pinch = min(TTD, DCA);
  assert(pinch > 0, "A negative pinch is reached", AssertionLevel.warning); // Ensure a positive pinch
  assert(pinch > 1 or pinch < 0,  "A very low pinch (<1) is reached", AssertionLevel.warning); // Ensure a sufficient pinch

  // Internal leaks
  tube_rupture.Q = 1e-5 + tube_rupture_leak;

  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{6,-64},{0,-64},{0,-80}},
      color={28,108,200},
      thickness=1));
  connect(C_hot_in, hot_side_pipe.C_in) annotation (Line(
      points={{-160,0},{-140,0}},
      color={238,46,47},
      thickness=1));

  connect(cold_side_condensing.C_out, cold_side_deheating.C_in) annotation (
      Line(
      points={{39,18},{39,30}},
      color={28,108,200},
      thickness=1));

  connect(tube_rupture.C_out, cold_side_deheating.C_out) annotation (Line(points={{14,38},{28,38},{28,62},{39,62},{39,60}}, color={217,67,180}));
  connect(tube_rupture.C_in, hot_side_deheating.C_in) annotation (Line(points={{-6,38},{-12,38},{-12,66},{-34,66},{-34,60}}, color={217,67,180}));
  connect(C_hot_in, C_hot_in)
    annotation (Line(points={{-160,0},{-160,0}}, color={28,108,200}));
  connect(cold_side_pipe.C_out, cold_side_vaporising.C_in)
    annotation (Line(points={{26,-64},{39,-64},{39,-60}},
                                                    color={28,108,200},
      thickness=1));
  connect(cold_side_vaporising.C_out, cold_side_condensing.C_in)
    annotation (Line(points={{39,-26},{39,-16}},   color={28,108,200},
      thickness=1));
  connect(hot_side_vaporising.C_in, hot_side_condensing.C_out)
    annotation (Line(points={{-34,-28},{-34,-16}},        color={238,46,47},
      thickness=1));
  connect(hot_side_vaporising.C_out, C_hot_out) annotation (Line(
      points={{-34,-60},{-34,-94},{100,-94},{100,0},{160,0}},
      color={238,46,47},
      thickness=1));
  connect(C_vent, hot_side_condensing.C_in) annotation (Line(points={{160,-78},{
          160,-80},{140,-80},{140,-100},{-60,-100},{-60,24},{-34,24},{-34,18}},
        color={238,46,47}));
  connect(hot_side_deheating.C_out, hot_side_condensing.C_in) annotation (Line(
      points={{-34,28},{-34,18}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_pipe.C_out, hot_side_deheating.C_in) annotation (Line(
      points={{-120,0},{-80,0},{-80,66},{-34,66},{-34,60}},
      color={238,46,47},
      thickness=1));
  connect(final_mix_cold.C_out, C_cold_out) annotation (Line(
      points={{4,62},{0,62},{0,80}},
      color={28,108,200},
      thickness=1));
  connect(final_mix_cold.C_in, cold_side_deheating.C_out) annotation (Line(
      points={{24,62},{39,62},{39,60}},
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
          points={{-98,60},{-98,38},{-98,-42},{-98,-62},{-78,-62},{14,-62},{142,-62},{142,60},{10,60},{-78,60},{-98,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{4,-26.75},{-4,-26.75},{-4,7.25},{-8,7.25},{0,22.75},{8,7.25},{4,7.25},{4,-26.75}},
          lineThickness=1,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-108,40},{-116,42},{-116,-46},{-108,-46},{-98,-46},{16,-46},{
              124,-46},{126,40},{12,40},{-100,40},{-108,40}},
          lineColor={238,46,47},
          smooth=Smooth.Bezier,
          lineThickness=1,
          pattern=LinePattern.Dash),
        Polygon(
          points={{-114,46},{-120,46},{-120,-52},{-112,-52},{-100,-52},{10,-52},
              {132,-52},{132,46},{10,46},{-100,46},{-114,46}},
          lineColor={238,46,47},
          smooth=Smooth.Bezier,
          lineThickness=1,
          pattern=LinePattern.Dash),
        Polygon(
          points={{-114,28},{-120,28},{-120,-34},{-112,-34},{-100,-34},{14,-34},
              {112,-34},{110,26},{12,28},{-100,28},{-114,28}},
          lineColor={238,46,47},
          smooth=Smooth.Bezier,
          lineThickness=1,
          pattern=LinePattern.Dash),
        Polygon(
          points={{-118,34},{-124,34},{-124,-38},{-116,-40},{-106,-40},{6,-40},{
              116,-42},{120,32},{8,34},{-104,34},{-118,34}},
          lineColor={238,46,47},
          smooth=Smooth.Bezier,
          lineThickness=1,
          pattern=LinePattern.Dash),
        Rectangle(
          extent={{-148,50},{-98,-56}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-146,-62},{-146,-38},{-146,-22},{-146,-22},{-120,-22},{-100,-22},{-100,-22},{-100,-62},{-100,-62},{-118,-62},{-146,-62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-146,60},{-146,36},{-146,20},{-146,20},{-120,20},{-100,20},{-100,20},{-100,60},{-100,60},{-118,60},{-146,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0})}),
                          Diagram(coordinateSystem(extent={{-160,-80},{160,80}}),
        graphics={                 Text(
          extent={{-20,4},{20,-4}},
          textColor={238,46,47},
          textString="Condensing",
          origin={-50,2},
          rotation=90),            Text(
          extent={{-20,4},{20,-4}},
          textColor={238,46,47},
          textString="Condensing",
          origin={-50,-44},
          rotation=90),            Text(
          extent={{-20,4},{20,-4}},
          textColor={238,46,47},
          origin={-50,44},
          rotation=90,
          textString="Deheating"), Text(
          extent={{-20,4},{20,-4}},
          textColor={28,108,200},
          origin={56,4},
          rotation=270,
          textString="Superheating"),
                                   Text(
          extent={{-20,4},{20,-4}},
          textColor={28,108,200},
          origin={56,-42},
          rotation=270,
          textString="Vaporising"),Text(
          extent={{-20,4},{20,-4}},
          textColor={28,108,200},
          origin={56,48},
          rotation=270,
          textString="Superheating"),
                                   Text(
          extent={{-20,4},{20,-4}},
          textColor={238,46,47},
          origin={-66,-90},
          rotation=90,
          textString="Vent")}));
end Superheater;
