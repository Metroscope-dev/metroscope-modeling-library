within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_Sunil_wEvap_Test

  //input Utilities.Units.Pressure P_FW_source(start=121);

  output Utilities.Units.MassFlowRate Q_FW_source(start = 50);
  input Real T_FW_source(start = 244.88, min = 0, nominal = 150);
  output Real Q_s(start = 50);
  //input Real W_evap(start=118.27767e6);

  input Real T_fg_source(start = 400);
  input Real P_fg_source(start = 1.1);
  input Real Q_fg(start = 650);

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Sunil_2                    Evap(
    V_D=40,
    M_D=140000,
    Cp_D=472)                                                                         annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=10e6, startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=10e6,
    duration=60,
    startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
  WaterSteam.BoundaryConditions.Sink d_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={44,-20})));
  WaterSteam.BoundaryConditions.Source r_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-2,-26})));
  HeatExchangers.TwoPhaseHX.Evaporator_3 evaporator_3_1(
    D_out=38e-3,
    e=2.6e-3,                                           N_tubes_row=1560, Rows=1,
    S_f=1/245)                                          annotation (Placement(transformation(extent={{-16,-66},{4,-46}})));
  FlueGases.BoundaryConditions.Source fg_source annotation (Placement(transformation(extent={{-56,-70},{-36,-50}})));
  FlueGases.BoundaryConditions.Sink fg_sink annotation (Placement(transformation(extent={{22,-70},{42,-50}})));
equation

  FW_source.P_out = 85*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;
  Steam_sink.Q_in = Q_s;  //+ ramp.y
  //Evap.W_evap = W_evap + step.y;
  //Heat.W = W_evap;
  r_source.h_out = Evap.h_d;
  //Evap.Q_d = - Q_FW_source*18.29;
  Q_FW_source = Q_s;

  // Calibration
  //Evap.Q_cd = 10.5; // h_f
  //Evap.Q_dc = 1195; // A_dc
  //Evap.V_sd = 4.9; // V_0_sd

  // Flue gas source
  fg_source.T_out = T_fg_source + 273.15;
  fg_source.P_out = P_fg_source*1e5;
  fg_source.Q_out = - Q_fg;
  fg_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  Evap.Q_d = Q_s*3.5;
  Evap.Q_r = Evap.Q_d;

  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,14},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Evap.fw_in, FW_source.C_out) annotation (Line(
      points={{14.2,0},{70,0},{70,-62},{79,-62}},
      color={28,108,200},
      thickness=1));
  connect(Evap.downcomers_out, d_sink.C_in) annotation (Line(points={{6,-4},{6,-20},{39,-20}},
                                                                                      color={28,108,200}));
  connect(evaporator_3_1.water_outlet, Evap.risers_in) annotation (Line(points={{-10,-46},{-12,-46},{-12,0},{-2,0}}, color={28,108,200}));
  connect(evaporator_3_1.water_inlet, r_source.C_out) annotation (Line(points={{-2,-46},{-2,-31}}, color={28,108,200}));
  connect(evaporator_3_1.fg_inlet, fg_source.C_out) annotation (Line(points={{-14,-56},{-28,-56},{-28,-60},{-41,-60}}, color={95,95,95}));
  connect(evaporator_3_1.fg_outlet, fg_sink.C_in) annotation (Line(points={{2,-56},{14,-56},{14,-60},{27,-60}}, color={95,95,95}));
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
end SteamDrum_Astom_Sunil_wEvap_Test;
