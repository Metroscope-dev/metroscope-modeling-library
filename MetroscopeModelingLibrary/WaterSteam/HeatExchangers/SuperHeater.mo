within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Superheater
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputFrictionCoefficient Kfr_cold;

  // Deheating
  Units.Power W_deheating;

  // Condensation
  Units.Power W_condensing;
  parameter String HX_config="condenser";
  Inputs.InputArea S;
  Units.HeatExchangeCoefficient Kth;
  Units.SpecificEnthalpy h_vap_sat_hot(start=2e6);
  Units.SpecificEnthalpy h_liq_sat_hot(start=1e5);
  Units.Temperature Tsat_hot;

  // Vaporising
  Units.Power W_vaporising;
  Units.SpecificEnthalpy h_vap_sat_cold;
  Units.Temperature Tsat_cold;

  // Ventilation
  Units.PositiveMassFlowRate Q_vent;
  Units.PositiveMassFlowRate Q_vent_faulty;

  // Definitions
  Units.PositiveMassFlowRate Q_cold(
    start=Q_cold_0,
    nominal=Q_cold_0,
    min=1e-5);
  Units.PositiveMassFlowRate Q_hot(start=Q_hot_0, nominal=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out;
  Units.Power W_tot;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage
  Units.Percentage closed_vent(min = 0, max= 100); // Vent closing percentage

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Temperature T_hot_in_0 = 273.15 + 220;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 100;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(extent={{-170,-10},{-150,10}}),iconTransformation(extent={{-170,-10},{-150,10}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0)) annotation (Placement(transformation(extent={{150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0)) annotation (Placement(transformation(extent={{-10,70},{10,90}}),iconTransformation(extent={{-10,70},{10,90}})));

  Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={16,-64})));
  Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-130,0})));
  BaseClasses.IsoPFlowModel hot_side_deheating(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={-34,44})));
  BaseClasses.IsoPFlowModel cold_side_deheating(Q_0=Q_cold_0) annotation (Placement(transformation(
        extent={{-15,-15},{15,15}},
        rotation=90,
        origin={39,45})));
  BaseClasses.IsoPFlowModel hot_side_condensing(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-17,-16},{17,16}},
        rotation=270,
        origin={-34,1})));
  BaseClasses.IsoPFlowModel cold_side_condensing(Q_0=Q_cold_0) annotation (Placement(transformation(
        extent={{-17,-17},{17,17}},
        rotation=90,
        origin={39,1})));
  Power.HeatExchange.NTUHeatExchange HX_condensing(config=HX_config, Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0,
                                                   T_hot_in_0=T_hot_in_0, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-21},{10,21}},
        rotation=270,
        origin={3,2})));
  BaseClasses.IsoPFlowModel cold_side_vaporising(Q_0=Q_cold_0) annotation (Placement(transformation(
        extent={{-17,-17},{17,17}},
        rotation=90,
        origin={39,-43})));
  BaseClasses.IsoPFlowModel hot_side_vaporising(Q_0=Q_cold_0) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={-34,-44})));
  Connectors.Outlet C_vent annotation (Placement(transformation(extent={{150,-88},{170,-68}})));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
    closed_vent = 0;
  end if;

  // Definitions
  Q_cold = cold_side_condensing.Q_in;
  Q_hot = hot_side_pipe.Q_in;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side_deheating.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side_vaporising.T_out;
  W_tot = W_deheating + W_condensing + W_vaporising;

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
  cold_side_deheating.W = W_deheating;

  // Power Exchange
  if hot_side_deheating.h_in > h_vap_sat_hot then
      hot_side_deheating.h_out = h_vap_sat_hot; // if steam is superheated, it is first deheated
  else
      hot_side_deheating.h_out = hot_side_deheating.h_in;
  end if;

  /* Condensing on hot side, superheating on cold side */

  // Energy Balance
  hot_side_condensing.W + cold_side_condensing.W = 0;
  cold_side_condensing.W = W_condensing;

  // Power Exchange
  HX_condensing.W = W_condensing;
  HX_condensing.Kth = Kth * (1 - fouling/100);
  HX_condensing.S = S;
  HX_condensing.Q_cold = Q_cold;
  HX_condensing.Q_hot = Q_hot - Q_vent;
  HX_condensing.T_cold_in = Tsat_cold;
  HX_condensing.T_hot_in = Tsat_hot;
  HX_condensing.Cp_cold = WaterSteamMedium.specificHeatCapacityCp(cold_side_condensing.state_in);
  HX_condensing.Cp_hot = 0; // Not used by NTU method in condenser mode

  /* Vaporising on cold side, condensation on hot side*/

  hot_side_vaporising.W + cold_side_vaporising.W = 0;
  W_vaporising = cold_side_vaporising.W;

  hot_side_vaporising.h_out = h_liq_sat_hot; // Hot steam is completely condensed

  if cold_side_vaporising.h_in < h_vap_sat_cold then
      cold_side_vaporising.h_out = h_vap_sat_cold;
  else
      cold_side_vaporising.h_out = cold_side_vaporising.h_in;
  end if;

  connect(cold_side_deheating.C_out, C_cold_out) annotation (Line(
      points={{39,60},{38,60},{38,66},{0,66},{0,80}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{6,-64},{0,-64},{0,-80}},
      color={28,108,200},
      thickness=1));
  connect(C_hot_in, hot_side_pipe.C_in) annotation (Line(
      points={{-160,0},{-140,0}},
      color={238,46,47},
      thickness=1));
  connect(hot_side_pipe.C_out, hot_side_deheating.C_in) annotation (Line(
      points={{-120,0},{-80,0},{-80,66},{-34,66},{-34,60}},
      color={238,46,47},
      thickness=1));

  connect(cold_side_condensing.C_out, cold_side_deheating.C_in) annotation (
      Line(
      points={{39,18},{39,30}},
      color={28,108,200},
      thickness=1));

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
