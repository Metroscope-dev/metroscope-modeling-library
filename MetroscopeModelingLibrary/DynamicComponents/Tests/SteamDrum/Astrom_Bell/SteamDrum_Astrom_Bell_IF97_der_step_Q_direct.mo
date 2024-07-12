within MetroscopeModelingLibrary.DynamicComponents.Tests.SteamDrum.Astrom_Bell;
model SteamDrum_Astrom_Bell_IF97_der_step_Q_direct

  /* Calibrated based on the steady-state results on the reference paper "Drum-boiler dynamics" by Astrom and Bell */
  parameter Boolean steady_state = false;

  // Calibrated BCs
  input Utilities.Units.MassFlowRate Q_f(start = 50.4195);
  input Real T_f(start = 241.87184, min = 0, nominal = 150); // 241.84042
  input Real Q_s(start = 50.4195);
  input Real W_evap(start=85.91237e6);

  // Inputs used for calibration
  output Real p(start=85e5); // W_evap
  output Real Q_dc(start=1194.5); // k
  output Real Q_ct(start=10.5); // T_f
  output Real V_sd(start=4.9); // V_0_sd
  output Real x_r(start=0.051); // Q_f and Q_s
  // V_wt is imposed in the

  // Calibrated parameters
  input Real k(start=25);
  input Real V_0_sd(start=7.6626515);

  .MetroscopeModelingLibrary.DynamicComponents.SteamDrum.Astrom_Bell.SteamDrum_Astrom_Bell_IF97_der Drum(
    steady_state=false,
    M_d=0,
    M_t=300000,
    M_r=160000,
    Cp=550,
    l_0=1.2002903,
    l_w_0=0.95529026,
    l_s_0=0.245) annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=10,   startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
equation

  // Boundary conditions
  FW_source.T_out = 273.15 + T_f;
  FW_source.Q_out = - Q_f;
  Steam_sink.Q_in = Q_s + step.y;
  Drum.W_evap = W_evap;

  // Inputs used for calibration
  Drum.p = p;
  Drum.Q_dc = Q_dc;
  Drum.Q_ct = Q_ct;
  Drum.V_sd = V_sd;
  Drum.x_r = x_r;

  // Calibrated parameters
  Drum.k = k;
  Drum.V_0_sd = V_0_sd;

  connect(Drum.steam_out, Steam_sink.C_in) annotation (Line(
      points={{0,14},{0,60},{-79,60}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(Drum.fw_in, FW_source.C_out) annotation (Line(
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
end SteamDrum_Astrom_Bell_IF97_der_step_Q_direct;
