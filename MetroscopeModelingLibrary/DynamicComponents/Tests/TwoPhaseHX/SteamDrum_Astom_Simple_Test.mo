within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX;
model SteamDrum_Astom_Simple_Test

  //input Utilities.Units.Pressure P_FW_source(start=121);

  output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
  input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
  output Real Q_s(start = 84.06);
  input Real W_evap(start=104.60049e6);

  HeatExchangers.TwoPhaseHX.SteamDrum_Astom_Simple
                                      steamDrum_Astom_Simple(Controlled_level=false)
                                                annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-60})));
  Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=10e6,
    duration=60,
    startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
equation

  FW_source.P_out = 121*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;
  Steam_sink.Q_in = Q_s;  //+ ramp.y
  steamDrum_Astom_Simple.W_evap = W_evap + step.y;  //+ step.y
  Q_s = 84.06/104.6649e6*steamDrum_Astom_Simple.W_evap;
  //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;

  connect(steamDrum_Astom_Simple.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,14},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steamDrum_Astom_Simple.fw_in, FW_source.C_out) annotation (Line(
      points={{14.2,0},{70,0},{70,-60},{79,-60}},
      color={28,108,200},
      thickness=1));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor={0,140,72},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,140,72},
                fillColor={0,140,72},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
end SteamDrum_Astom_Simple_Test;
