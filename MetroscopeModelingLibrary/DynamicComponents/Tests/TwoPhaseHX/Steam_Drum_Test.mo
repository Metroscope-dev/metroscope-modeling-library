within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX;
model Steam_Drum_Test

  output Utilities.Units.Pressure P_FW_source;
  output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
  input Real T_FW_source(start = 315.6, min = 130, nominal = 150);

  HeatExchangers.TwoPhaseHX.SteamDrum steamDrum annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,20})));
  WaterSteam.Pipes.HeatLoss evaporator annotation (Placement(transformation(extent={{-20,-50},{-40,-30}})));
equation

  FW_source.P_out = P_FW_source*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;

  connect(steamDrum.steam_out, Steam_sink.C_in) annotation (Line(
      points={{-6,58},{-6,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steamDrum.fw_in, FW_source.C_out) annotation (Line(
      points={{8.2,44},{34,44},{34,20},{79,20}},
      color={28,108,200},
      thickness=1));
  connect(evaporator.C_in, steamDrum.downcomers_out) annotation (Line(
      points={{-20,-40},{0,-40},{0,40}},
      color={28,108,200},
      thickness=1));
  connect(evaporator.C_out, steamDrum.risers_in) annotation (Line(
      points={{-40,-40},{-60,-40},{-60,44},{-8,44}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor={0,140,72},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,140,72},
                fillColor={0,140,72},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Steam_Drum_Test;
