within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_6_Ahmed_direct

  //input Utilities.Units.Pressure P_FW_source(start=121);

  input Utilities.Units.MassFlowRate Q_FW_source(start = 60.9);
  input Real T_FW_source(start = 278.72903, min = 0, nominal = 150);
  input Real Q_s(start = 60.9);
  input Real W_evap(start=9.339889e+07);

  HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_quad_functions_6
                                                                                 Evap(
    V_D=45,
    M_d=88600,
    M_t=465000,
    V_r=43,
    M_r=370000,
    Cp=550,
    A_d=24,
    V_dc=9.5,
    k_friction=310,
    beta=0.3,
    T_d=12,
    V_wt_0=50)                                                                        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=12e6, startTime=150)
                                                               annotation (Placement(transformation(extent={{42,34},{62,54}})));
equation

  //FW_source.P_out = 78*1e5; // W_evap

  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;

  Steam_sink.Q_in = Q_s;  //+ ramp.y
  Evap.W_evap = W_evap + step.y; // + step.y

  Evap.A_dc = 0.69720966;
  Evap.V_0_sd = 9.89967;
  //Evap.V_sd = 9;
  //Evap.Q_dc = 615;
  //Evap.x_r = 0.104;
  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
      points={{-2,8},{-2,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Evap.fw_in, FW_source.C_out) annotation (Line(
      points={{12.2,-6},{70,-6},{70,-62},{79,-62}},
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
end SteamDrum_Astom_6_Ahmed_direct;
