within MetroscopeModelingLibrary.DynamicComponents.Tests.SteamDrum.Astrom_Bell;
model SteamDrum_Astrom_Bell_wHX

  parameter Boolean steady_state = false;
  // Calibrated BCs
  input Utilities.Units.MassFlowRate Q_f(start = 50.4195);
  input Real T_f(start = 241.87184, min = 0, nominal = 150); // 241.84042
  input Real Q_s(start = 10.4195);
  input Real T_fg_source(start = 400);
  input Real P_fg_source(start = 1.1);
  input Real Q_fg(start = 650);

  // Inputs used for calibration
  output Real p(start=85e5); // W_evap
  output Real Q_dc(start=1194.5); // A_dc
  output Real Q_ct(start=10.5); // T_f
  output Real V_sd(start=4.9); // V_0_sd
  output Real x_r(start=0.051); // Q_f and Q_s
  // V_wt is imposed in reverse mode

  // Calibrated parameters
  input Real k(start=25);
  input Real V_0_sd(start=7.6626515);
  input Real eta_fins(start=0.07966796);

  // Outputs of interest
  output Real W_evap(start=85.91237e6);

  WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,46},{-94,66}})));
  WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={84,-62})));
  Modelica.Blocks.Sources.Step step(height=0.1,  startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
  MetroscopeModelingLibrary.DynamicComponents.SteamDrum.Astrom_Bell.SteamDrum_Astrom_Bell_wHX
                                                                                     Drum(steady_state=steady_state, A_dc=0.38128173)
                                                                                                                     annotation (Placement(transformation(extent={{-10,-20},{10,22}})));
  FlueGases.BoundaryConditions.Source fg_source annotation (Placement(transformation(extent={{-54,-20},{-34,0}})));
  FlueGases.BoundaryConditions.Sink fg_sink annotation (Placement(transformation(extent={{34,-20},{54,0}})));
  WaterSteam.BoundaryConditions.Sink Water_sink annotation (Placement(transformation(extent={{-36,-4},{-56,16}})));
equation

  // Boundary conditions
  FW_source.T_out = 273.15 + T_f;
  FW_source.Q_out = - Q_f;
  Steam_sink.Q_in = Q_s;
  fg_source.T_out = T_fg_source + 273.15;
  fg_source.P_out = P_fg_source*1e5;
  fg_source.Q_out = - Q_fg - step.y*Q_fg;
  fg_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  // Inputs used for calibration
  Drum.p = p;
  Drum.Q_dc = Q_dc;
  Drum.Q_ct = Q_ct;
  Drum.V_sd = V_sd;
  Drum.x_r = x_r;
  //Drum.V_wt = 57.2;
  Drum.Q_f = Drum.Q_s + Drum.Q_w_out;

  // Calibrated parameters
  Drum.k = k;
  Drum.V_0_sd = V_0_sd;
  Drum.eta_fins = eta_fins;

  // Outputs of interest
  Drum.W_evap = W_evap;

  connect(Drum.fw_in, FW_source.C_out) annotation (Line(points={{8.2,6},{60,6},{60,-62},{79,-62}}, color={28,108,200}));
  connect(Drum.steam_out, Steam_sink.C_in) annotation (Line(points={{-6,20},{-6,56},{-79,56}},          color={28,108,200}));
  connect(Drum.fg_inlet, fg_source.C_out) annotation (Line(points={{-8,-10},{-24,-10},{-24,-10},{-39,-10}},
                                                                                        color={95,95,95}));
  connect(Drum.fg_outlet, fg_sink.C_in) annotation (Line(points={{8,-10},{24,-10},{24,-10},{39,-10}},
                                                                                    color={95,95,95}));
  connect(Water_sink.C_in,Drum. water_out) annotation (Line(points={{-41,6},{-8,6}},                     color={28,108,200}));
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
end SteamDrum_Astrom_Bell_wHX;
