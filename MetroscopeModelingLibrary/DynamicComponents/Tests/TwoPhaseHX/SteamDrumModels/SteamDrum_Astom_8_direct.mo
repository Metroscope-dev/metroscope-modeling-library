within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_8_direct

  //input Utilities.Units.Pressure P_FW_source(start=121);

  input Utilities.Units.MassFlowRate Q_FW_source(start = 49.9195); // 49.9195 39.784206
  input Real T_FW_source(start = 238.25188, min = 0, nominal = 150); // 241.84042
  output Real Q_s(start = 60);
  input Real W_evap(start=85.91238e6); // 85.91238e6

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_8 Evap(
    M_d=0,
    M_t=300000,
    M_r=160000,
    Cp=550,
    beta=0.3,
    l_0=0,
    l_w_0=0,
    l_s_0=0)                                                                          annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
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
equation

  //FW_source.P_out = 85*1e5;
  //FW_source.T_out = Evap.T_wall - 60;
  FW_source.T_out = T_FW_source + 273.15;
  FW_source.Q_out = - Q_FW_source;
  //FW_source.h_out = 162.6;

  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap + step.y;  // + step.y

  //Evap.Q_ct = 10.5; // h_f
  //Evap.Q_dc = 1194; // A_dc
  Evap.A_dc = 0.38128173;
  //Evap.V_sd = 4.9; // V_0_sd
  Evap.V_0_sd = 7.794206; // 7.794206
  FW_source.Q_out  =- Q_s;
  //Evap.x_r = 0.051; // T_FW
  //Evap.l = 1.2;

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
      __Dymola_fixedstepsize=0.0001,
      __Dymola_Algorithm="Dassl"));
end SteamDrum_Astom_8_direct;
