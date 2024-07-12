within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_7_Ahmed_reverse

  //input Utilities.Units.Pressure P_FW_source(start=121);

  output Utilities.Units.MassFlowRate Q_FW_source(start = 50.445);
  output Real T_FW_source(start = 241.90323, min = 0, nominal = 150);
  output Real Q_s(start = 50.445);
  output Real W_evap(start=85.94833e6);

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_7 Evap(
    V_D=45,
    M_t=465000,
    V_r=43,
    M_r=370000,
    Cp=550,
    A_dc=0.63,
    A_d=24,
    V_dc=9.5,
    k_friction=310,
    beta=0.3,
    T_d=12,
    V_wt_0=50)                                                                        annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
equation

  FW_source.P_out = 78*1e5; // W_evap

  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;

  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap; // + step.y

  Evap.V_sd = 9;
  Evap.x_r = 0.104;

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
    experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
end SteamDrum_Astom_7_Ahmed_reverse;
