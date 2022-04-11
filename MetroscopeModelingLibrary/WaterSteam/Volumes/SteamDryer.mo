within MetroscopeModelingLibrary.WaterSteam.Volumes;
model SteamDryer

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;

  // Initialization parameters
  parameter Units.Pressure P_0 = 10e5;
  parameter Units.InletMassFlowRate Q_in_0=500;

  Units.SpecificEnthalpy h_vap_sat(start=h_vap_sat_0); // Saturated liquid enthalpy
  Units.SpecificEnthalpy h_liq_sat(start=h_liq_sat_0); // Saturated steam enthalpy

  Units.Pressure P(start=P_0); // Pressure in dryer
  Units.InletMassFlowRate Q_in(start=Q_in_0); // Inlet mass flow rate

  Units.MassFraction x_steam_out; // Steam mass fraction at steam outlet

  Connectors.Inlet C_in(P(start=P_0), Q(start=Q_in_0)) annotation (Placement(transformation(extent={{-110,30},{-90,50}}), iconTransformation(extent={{-110,30},{-90,50}})));
  Connectors.Outlet C_hot_steam(P(start=P_0), Q(start=-Q_in_0/2)) annotation (Placement(transformation(extent={{90,30},{110,50}})));
  Connectors.Outlet C_hot_liquid(P(start=P_0), Q(start=-Q_in_0/2)) annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  WaterSteam.BaseClasses.WaterIsoPFlowModel steam_phase(P_0=P_0, Q_0=Q_in_0/2)
    annotation (Placement(transformation(extent={{26,30},{46,50}})));
  WaterSteam.BaseClasses.WaterIsoPFlowModel liquid_phase(P_0=P_0, Q_0=Q_in_0/2)
    annotation (Placement(transformation(extent={{26,-50},{46,-30}})));
protected
  parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_0));
  parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_0));
equation

  // Definitions
  P = C_in.P;
  Q_in = steam_phase.Q_in + liquid_phase.Q_in;

  // Saturation at both outlets
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));
  steam_phase.h_out = x_steam_out * h_vap_sat + (1-x_steam_out)*h_liq_sat;
  liquid_phase.h_out = h_liq_sat;

  // Energy balance
  steam_phase.W + liquid_phase.W = 0;

  connect(liquid_phase.C_in, C_in) annotation (Line(points={{26,-40},{-40,-40},{
          -40,40},{-100,40}},
                            color={28,108,200}));
  connect(steam_phase.C_in, C_in) annotation (Line(points={{26,40},{-40,40},{-40,
          40},{-100,40}},
                        color={28,108,200}));
  connect(steam_phase.C_out,C_hot_steam)
    annotation (Line(points={{46,40},{100,40}}, color={28,108,200}));
  connect(liquid_phase.C_out,C_hot_liquid)
    annotation (Line(points={{46,-40},{100,-40}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,120}}), graphics={
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,0},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Line(
          points={{-70,40},{-70,-20}},
          color={64,82,185},
          thickness=1),
        Polygon(
          points={{-100,-34},{100,-20},{100,-40},{-100,-40},{-100,-34}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-40,22},{-40,-40}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{-10,40},{-10,-20}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{20,22},{20,-40}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{50,40},{50,-20}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{80,18},{80,-40}},
          color={64,82,185},
          thickness=1),
        Ellipse(
          extent={{-86,28},{-76,18}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-82,-4},{-72,-14}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-56,-12},{-46,-22}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-54,34},{-44,24}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-26,-8},{-16,-18}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{6,32},{16,22}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{32,-4},{42,-14}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{52,28},{62,18}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-36,22},{-26,12}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-64,10},{-54,0}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={64,82,185},
          lineThickness=1)}),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,120}})));
end SteamDryer;
