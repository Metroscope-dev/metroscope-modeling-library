within MetroscopeModelingLibrary.Partial.Volumes;
model PhaseSeparationVolume

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Pressure P_0 = 10e5;
  parameter Units.PositiveMassFlowRate Q_in_0=500;
  parameter Units.PositiveMassFlowRate Q_liq_0 = 0.5*Q_in_0;
  parameter Units.PositiveMassFlowRate Q_vap_0 = Q_in_0 - Q_liq_0;

  // Inlet
  Units.Pressure P(start=P_0);
  Units.PositiveMassFlowRate Q_in(start=Q_in_0);

  // Saturation
  Units.SpecificEnthalpy h_vap_sat(start=h_vap_sat_0);
  Units.SpecificEnthalpy h_liq_sat(start=h_liq_sat_0);

  // Outlet
  Units.MassFraction x_steam_out(start=1); // Steam mass fraction at steam outlet

  WaterSteam.Connectors.Inlet C_in(P(start=P_0), Q(start=Q_in_0)) annotation (Placement(transformation(extent={{-110,30},{-90,50}}), iconTransformation(extent={{-110,30},{-90,50}})));
  WaterSteam.Connectors.Outlet C_steam_out(
    P(start=P_0),
    Q(start=-Q_vap_0),
    h_outflow(start=h_vap_sat_0)) annotation (Placement(transformation(extent={{90,30},{110,50}})));
  WaterSteam.Connectors.Outlet C_liquid_out(
    P(start=P_0),
    Q(start=-Q_liq_0),
    h_outflow(start=h_liq_sat_0)) annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_0));

equation
  // Inlet
  C_in.h_outflow = 0;

  // Definitions
  P = C_in.P;
  Q_in = C_in.Q;
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));

  // Balances
  Q_in * inStream(C_in.h_outflow) + C_steam_out.Q * C_steam_out.h_outflow + C_liquid_out.Q * C_liquid_out.h_outflow = 0; // Energy balance
  Q_in + C_steam_out.Q + C_liquid_out.Q = 0; // Mass balance

  // Outlet
  C_steam_out.P = P;
  C_liquid_out.P = P;

  C_steam_out.h_outflow = x_steam_out * h_vap_sat + (1-x_steam_out)*h_liq_sat;
  C_liquid_out.h_outflow = h_liq_sat;

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
end PhaseSeparationVolume;
