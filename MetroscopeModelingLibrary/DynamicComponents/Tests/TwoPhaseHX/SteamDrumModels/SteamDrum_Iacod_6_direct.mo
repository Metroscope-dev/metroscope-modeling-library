within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Iacod_6_direct

  //input Utilities.Units.Pressure P_FW_source(start=121);

  input Utilities.Units.MassFlowRate Q_FW_source(start = 14.8375); // 49.9195 39.784206
  input Real T_FW_source(start = 83.41566, min = 0, nominal = 150); // 241.84042
  output Real Q_s(start = 60);
  input Real W_evap(start=36173516);

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_5_debug
                                                                                 Evap(
    V_D=31.4,
    M_t=18000,
    V_r=15,
    M_r=16000,
    Cp=650,
    A_d=20,
    V_dc=11.41,
    k_friction=25,
    beta=0.3,
    T_d=12,
    V_wt_0=19,
    l_0=0,
    l_w_0=0,
    l_s_0=0)                                                                          annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=0.5e6,startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=10e6,
    duration=60,
    startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
equation

  //FW_source.P_out = 13.75*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;
  //FW_source.h_out = 162.6;

  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap + step.y;  // + step.y

  //Evap.Q_ct = 3.6; // h_f
  //Evap.Q_dc = 1475; // A_dc
  Evap.A_dc = 0.6153176;
  Evap.V_sd = 3.82; // V_0_sd
  //Evap.V_0_sd = 7.794206; // 7.662651
  FW_source.Q_out = - Q_s;
  //Evap.x_r = 0.0125; // T_FW
  Evap.A_ma = 6.7092476;

  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,14},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Evap.fw_in, FW_source.C_out) annotation (Line(
      points={{14.2,0},{70,0},{70,-62},{79,-62}},
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
    experiment(
      StopTime=200,
      __Dymola_NumberOfIntervals=200,
      __Dymola_Algorithm="Dassl"));
end SteamDrum_Iacod_6_direct;
