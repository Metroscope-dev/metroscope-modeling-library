within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_5_Xuandai_direct

  //input Utilities.Units.Pressure P_FW_source(start=121);

  output Utilities.Units.MassFlowRate Q_FW_source(start = 2); // 50.445
  input Real T_FW_source(start = 426-273.15, min = 0, nominal = 150); // 241.90323
  output Real Q_s(start = 60);
  input Real W_evap(start=4.28e6);

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_5 Evap(
    V_D=2.27,
    M_d=7320,
    M_t=21570,
    V_r=1.5,
    M_r=11210,
    Cp=550,
    A_dc=0.052,
    A_d=2.7,
    V_dc=0.73,
    k_friction=103,
    beta=0.3,
    T_d=3,
    p_0=4500000,
    V_wt_0=2.2,
    l_0=0,
    l_w_0=0,
    l_s_0=0,
    V_sd(start=0.189),
    x_r(start=0.0248))                                                                annotation (Placement(transformation(extent={{-4,-2},{16,18}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=0.1,  startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=10e6,
    duration=60,
    startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
equation

  //FW_source.P_out = 45*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;

  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap + step.y*W_evap; // + step.y

  //Evap.Q_cd = 10.5; // h_f
  //Evap.Q_dc = 59.4; // A_dc
  //Evap.A_dc = 59.4;
  Evap.V_sd = 0.189; // V_0_sd
  Evap.V_0_sd = 0.264; // 7.662651 // 264
  Q_FW_source = Q_s;
  //Evap.x_r = 0.0248; // T_FW
  Evap.A_ma = 6.7092476;

  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,16},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Evap.fw_in, FW_source.C_out) annotation (Line(
      points={{14.2,2},{70,2},{70,-62},{79,-62}},
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
end SteamDrum_Astom_5_Xuandai_direct;
