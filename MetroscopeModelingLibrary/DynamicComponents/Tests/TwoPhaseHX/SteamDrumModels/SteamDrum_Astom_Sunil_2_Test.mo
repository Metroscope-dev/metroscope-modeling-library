within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_Sunil_2_Test

  //input Utilities.Units.Pressure P_FW_source(start=121);

  input Utilities.Units.MassFlowRate Q_FW_source(start = 70);
  input Real T_FW_source(start = 244.88, min = 0, nominal = 150);
  input Real Q_s(start = 70);
  input Real W_evap(start=118.27767e6);

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
        rotation=90,
        origin={6,-44})));
  WaterSteam.BoundaryConditions.Source r_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-44})));
  WaterSteam.Pipes.HeatLoss Heat annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-16})));
equation

  FW_source.P_out = 85*1e5;
  FW_source.T_out = 273.15 + T_FW_source;
  FW_source.Q_out = - Q_FW_source;
  Steam_sink.Q_in = Q_s;  //+ ramp.y
  //Evap.W_evap = W_evap + step.y;
  Heat.W = W_evap;
  Heat.h_in = Evap.h_d;
  //Evap.Q_d = - Q_FW_source*18.29;

  // Calibration
  //Evap.Q_cd = 10.5; // h_f
  //Evap.Q_dc = 1195; // A_dc
  //Evap.V_sd = 4.9; // V_0_sd

  connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,14},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Evap.fw_in, FW_source.C_out) annotation (Line(
      points={{14.2,0},{70,0},{70,-62},{79,-62}},
      color={28,108,200},
      thickness=1));
  connect(Evap.downcomers_out, d_sink.C_in) annotation (Line(points={{6,-4},{6,-39}}, color={28,108,200}));
  connect(r_source.C_out, Heat.C_in) annotation (Line(points={{-20,-39},{-20,-26}}, color={28,108,200}));
  connect(Heat.C_out, Evap.risers_in) annotation (Line(points={{-20,-6},{-20,0},{-2,0}}, color={28,108,200}));
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
end SteamDrum_Astom_Sunil_2_Test;
