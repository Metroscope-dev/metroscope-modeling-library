within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_SimplifiedGeometry_wHX_liquid_out_SS_Test

  //input Utilities.Units.Pressure P_FW_source(start=121);

  input Utilities.Units.MassFlowRate Q_FW_source(start = 120);
  input Real T_FW_source(start = 244.88, min = 0, nominal = 150);
  output Real Q_s(start = 70);
  output Real W_evap(start=118.27767e6);
  output Real p(start=85);
  input Real Q_w(start=50);

  input Real T_fg_source(start = 400);
  input Real P_fg_source(start = 1.1);
  input Real Q_fg(start = 650);

  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,46},{-94,66}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=20,   startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=10e6,
    duration=60,
    startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_wHX_liquid_out_SS
                                                                                     Evap(steady_state=true)
                                                                                          annotation (Placement(transformation(extent={{-10,-20},{10,22}})));
  FlueGases.BoundaryConditions.Source fg_source annotation (Placement(transformation(extent={{-54,-20},{-34,0}})));
  FlueGases.BoundaryConditions.Sink fg_sink annotation (Placement(transformation(extent={{34,-20},{54,0}})));
  WaterSteam.BoundaryConditions.Sink Water_sink annotation (Placement(transformation(extent={{-60,-6},{-80,14}})));
equation

  FW_source.P_out = p*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;
  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap;

  // Flue gas source
  fg_source.T_out = T_fg_source + 273.15;
  fg_source.P_out = P_fg_source*1e5;
  fg_source.Q_out = - Q_fg + step.y;
  fg_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  Evap.eff_fins = 0.77;
  Evap.Q_w_out = Q_w;

  connect(Evap.fw_in, FW_source.C_out) annotation (Line(points={{8.2,6},{60,6},{60,-62},{79,-62}}, color={28,108,200}));
  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(points={{-6,20},{-6,56},{-79,56}},          color={28,108,200}));
  connect(Evap.fg_inlet, fg_source.C_out) annotation (Line(points={{-8,-10},{-39,-10}}, color={95,95,95}));
  connect(Evap.fg_outlet, fg_sink.C_in) annotation (Line(points={{8,-10},{39,-10}}, color={95,95,95}));
  connect(Water_sink.C_in, Evap.water_out) annotation (Line(points={{-65,4},{-36,4},{-36,6},{-8,6}},     color={28,108,200}));
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
    experiment(StopTime=200, __Dymola_Algorithm="Dassl"));
end SteamDrum_Astom_SimplifiedGeometry_wHX_liquid_out_SS_Test;
