within MetroscopeModelingLibrary.WaterSteam.Volumes;
model FlashTank

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Pressure P_0 = 10e5;
  parameter Units.Temperature T_0 = 273.15 + 180;
  parameter Units.SpecificEnthalpy h_in_0 = 2e6;
  parameter Units.PositiveMassFlowRate Q_in_0=500;
  parameter Units.PositiveMassFlowRate Q_liq_0 = 0.5*Q_in_0;
  parameter Units.PositiveMassFlowRate Q_vap_0 = Q_in_0 - Q_liq_0;


  Units.Pressure P(start=P_0);
  Units.PositiveMassFlowRate Q_in(start=Q_in_0);

  Connectors.Inlet C_in(P(start=P_0), Q(start=Q_in_0)) annotation (Placement(transformation(extent={{-110,30},{-90,50}}), iconTransformation(extent={{-110,30},{-90,50}})));
  Connectors.Outlet C_hot_steam(P(start=P_0), Q(start=-Q_vap_0),
    h_outflow(start=h_vap_sat_0))                                 annotation (Placement(transformation(extent={{90,30},{110,50}})));
  Connectors.Outlet C_hot_liquid(P(start=P_0), Q(start=-Q_liq_0),
    h_outflow(start=h_liq_sat_0))                                  annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  BaseClasses.IsoPFlowModel steam_phase(
    T_in_0=T_0,
    T_out_0=T_0,
    h_in_0=h_in_0,
    h_out_0=h_vap_sat_0,                P_0=P_0,
    Q_0=Q_vap_0)                                               annotation (Placement(transformation(extent={{26,30},{46,50}})));
  BaseClasses.IsoPFlowModel liquid_phase(
    T_in_0=T_0,
    T_out_0=T_0,
    h_in_0=h_in_0,
    h_out_0=h_liq_sat_0,                 P_0=P_0,
    Q_0=Q_liq_0)                                                annotation (Placement(transformation(extent={{26,-50},{46,-30}})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_0));

equation

  // Definitions
  P = C_in.P;
  Q_in = steam_phase.Q_in + liquid_phase.Q_in;

  // Saturation at both outlets
  steam_phase.h_out = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
  liquid_phase.h_out = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));

  // Energy balance
  steam_phase.W + liquid_phase.W = 0;

  connect(steam_phase.C_in, C_in)
    annotation (Line(points={{26,40},{-100,40}}, color={28,108,200}));
  connect(steam_phase.C_out, C_hot_steam)
    annotation (Line(points={{46,40},{100,40}}, color={28,108,200}));
  connect(liquid_phase.C_out, C_hot_liquid)
    annotation (Line(points={{46,-40},{100,-40}}, color={28,108,200}));
  connect(liquid_phase.C_in, C_in) annotation (Line(points={{26,-40},{-28,-40},
          {-28,40},{-100,40}}, color={28,108,200}));
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-100,-10},{100,-10},{100,-40},{-100,-40},{-100,-10}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-88,22},{-82,16}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-80,10},{-74,4}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-74,26},{-68,20}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-92,0},{-86,-6}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-84,34},{-78,28}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-68,6},{-62,0}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-60,22},{-54,16}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-48,10},{-42,4}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-54,-2},{-48,-8}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-94,14},{-88,8}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-28,0},{-22,-6}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-76,0},{-70,-6}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          lineThickness=1)}));
end FlashTank;
