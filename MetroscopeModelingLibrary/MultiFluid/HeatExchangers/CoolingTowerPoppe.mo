within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTowerPoppe
  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-10,80},{10,100}})));
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

  function f
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real Qw;
    input Real Qa;
    input Real P_in;
    input Real Lef;
    output Real y;
  algorithm
    y:= (cp * (Qw / Qa) * (MoistAir.xsaturation_pT(P_in, Tw) - w)) / (((MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)})) - i + (Lef-1) * ((MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)}) - i - (MoistAir.xsaturation_pT(P_in, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(P_in, 100+273.15, 1))) - (MoistAir.xsaturation_pT(P_in, Tw) - w) * cp * Tw));
  end f;

  //Check specific enthalpy of steam mass fraction value, 1 or 0 ?
  //Check also if temperature is in kelvin or degrees for all three functions ?

  function g
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real Qw;
    input Real Qa;
    input Real P_in;
    input Real Lef;
    output Real y;
  algorithm
    y:= ((Qw * cp) / Qa) * (1 + (((MoistAir.xsaturation_pT(P_in, Tw) - w) * (cp * Tw)) / ((MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)}) - i + ((Lef-1) * (MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)}) - i - (MoistAir.xsaturation_pT(P_in, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(P_in, 100+273.15, 1))) - (MoistAir.xsaturation_pT(P_in, Tw) - w) * cp * Tw))));
  end g;



  function h
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real P_in;
    input Real Lef;
    output Real y;
  algorithm
    y:= cp / (MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)}) - i + (Lef-1) * (MoistAir.h_pTX(P_in, Tw, {MoistAir.massFraction_pTphi(P_in, Tw, 1)}) - i - (MoistAir.xsaturation_pT(P_in, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(P_in, 100+273.15, 1)) - (MoistAir.xsaturation_pT(P_in, Tw) - w) * cp * Tw);
  end h;


  Inputs.InputArea Afr;
  Inputs.InputReal Lfi;
  Inputs.InputReal afi;
  Inputs.InputReal hd;
  Inputs.InputReal D;
  Units.Velocity V_inlet;
  Inputs.InputFrictionCoefficient Cf;

  Units.MassFlowRate Q_cold_in;
  Units.MassFlowRate Q_cold_out;
  Units.MassFlowRate Q_hot_in;
  Units.MassFlowRate Qw[N_step];
  Units.MassFlowRate Qa[N_step];
  Units.MassFlowRate Q_hot_out;
  Units.MassFlowRate Q_makeup;

  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.SpecificEnthalpy i_initial;
  Units.SpecificEnthalpy i_final;

  Units.Power W;
  Real w_in;
  Real w_out;

  Units.Density rho_air_inlet(start=rho_air_inlet_0);
  Units.Density rho_air_outlet(start=rho_air_outlet_0);

  Units.HeatCapacity cp;
  Units.Pressure P_in;
  Units.Pressure P_out;

  constant Real gr(unit="m/s2") = Modelica.Constants.g_n;

    // Poppe Inputs
  Units.Temperature deltaT;
  Real Lef;                                    //Why is this an output
  parameter Integer N_step = 1;
  Real w[N_step];                               //Not sure if these should be output or Units.......
  Real M[N_step];
  Real i[N_step];
  Real Tw[N_step];

  // Initialization Parameters

  parameter Units.Temperature T_cold_in_0 = 15 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 25 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 40 + 273.15;
  parameter Units.Temperature T_hot_out_0 = 20 + 273.15;

  parameter Units.Density rho_air_inlet_0 = 1.2754;
  parameter Units.Density rho_air_outlet_0 = 1.2460;

  MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{-100,-10},{-80,10}}), iconTransformation(extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{-10,-100},{10,-80}})));
  WaterSteam.BaseClasses.IsoPHFlowModel                          hot_side_cooling annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink Air_inlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,18})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source Air_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-18})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel inputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,38})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel outputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-60})));
  MetroscopeModelingLibrary.MoistAir.Pipes.Pipe pipe annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,62})));
  WaterSteam.BoundaryConditions.Sink Water_inlet annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  WaterSteam.BoundaryConditions.Source Water_outlet annotation (Placement(transformation(extent={{10,-10},{30,10}})));
equation
  // Definition
  Q_cold_in = Air_inlet.Q_in;
  Q_cold_out = Air_outlet.Q_out;
  Q_hot_in = Water_inlet.Q_in;
  Q_hot_out = Water_outlet.Q_out;

  T_hot_in = Water_inlet.T_in;
  T_hot_out = Water_outlet.T_out;
  T_cold_in = Air_inlet.T_in;
  T_cold_out = Air_outlet.T_out;
  P_in = Air_inlet.P_in;
  P_out = Air_outlet.P_out;

  w_in = Air_inlet.relative_humidity * MoistAir.xsaturation(Air_inlet.state_in);
  w_out = Air_outlet.relative_humidity * MoistAir.xsaturation(Air_outlet.state_out);           //multiplication converts it to absolute humidity for both ?
  cp = WaterSteamMedium.specificHeatCapacityCp(hot_side_cooling.state_in);

  W = Q_hot_in * cp * (T_hot_in - T_hot_out);

  P_in - P_out = 0;
  Water_outlet.P_out = Water_inlet.P_in;

  i_initial = Air_inlet.h_in;
  i_final = Air_outlet.h_out;

  pipe.Kfr = 0;
  pipe.delta_z = 0;


  // New Poppe Equations

  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(P_in, T_cold_in)+0.622)/(w_in+0.622))-1) / log((MoistAir.xsaturation_pT(P_in, T_cold_in)+0.622)/(w_in+0.622));
  //Lef = hd /cp;

  deltaT = (T_hot_in - T_hot_out)/N_step;

  for n in 1:N_step-1 loop
    w[n+1] = w[n] + deltaT * f(Tw[n], w[n], i[n], cp, Qw[n], Qa[n], P_in, Lef);                               //Add absolute humidity conversion ?
    i[n+1] = i[n] + deltaT * g(Tw[n], w[n], i[n], cp, Qw[n], Qa[n], P_in, Lef);
    M[n+1]= M[n] + deltaT * h(Tw[n], w[n], i[n], cp, P_in, Lef);
    Qw[n] = Qw[n+1] + Qa[n] * (w[n+1] - w[n]);
    Qa[n+1] = Qa[n] * (1 + w[n+1]);
    M[n+1] = hd * Afr / Qw[n+1];
  end for;

  w[1] = w_in;
  w[N_step] = w_out;
  i[1] = i_initial;
  i[N_step] = i_final;
  Tw[1] = T_hot_in "degC";                    //added these to [N_step] equations to correlate with defined variables (so the models knows at the end of the loop its at the outlets)
  Tw[N_step] = T_hot_out "degC";
  M[1] = hd * Afr / Q_hot_in; //need start value for hd in loop or in the reverse model calibrate M[1] not hd ?
  Qw[N_step] = Q_hot_out;
  Qw[1] = Q_hot_in;
  Qa[1] = Q_cold_in;
  Qa[N_step] = Q_cold_out;

  // Drift Equation
  rho_air_inlet = inputflowmodel.rho_in;
  rho_air_outlet = outputflowmodel.rho_out;

  0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  =  (rho_air_inlet - rho_air_outlet) * gr * Lfi;
  Q_cold_in = (V_inlet * Afr * rho_air_inlet * (1 - Air_inlet.Xi_in[1]));




  //WaterSteamMedium.specificEnthalpy_pT(P_in, 100+273.15, 1)
  //MoistAirMedium.relativeHumidity_pTX(P_in, T[n], {MoistAir.massFraction_pTphi(P_in, T[n], 1)})
  //MoistAir.saturationPressure
  //MoistAir.xsaturation_pT
  //MoistAir.Xsaturation
  //w[1] = MoistAir.xsaturation(Air_inlet.state_in);

  //Absolute Humidity = Relative Humidity * Saturation Absolute Humidity at temperature
  //Absolute Humidity = Relative Humidity * Maximum Possible Humidity


  connect(C_hot_in, hot_side_cooling.C_in) annotation (Line(points={{-90,0},{-70,0}}, color={28,108,200}));
  connect(inputflowmodel.C_out, Air_inlet.C_in) annotation (Line(points={{0,28},{0,23}},                                                             color={85,170,255}));
  connect(Air_outlet.C_out, outputflowmodel.C_in) annotation (Line(points={{0,-23},{0,-50}},                                                               color={85,170,255}));
  connect(outputflowmodel.C_out, C_cold_out) annotation (Line(points={{0,-70},{0,-90}},                                       color={85,170,255}));
  connect(pipe.C_in, C_cold_in) annotation (Line(points={{1.77636e-15,72},{0,81},{0,90}},                  color={85,170,255}));
  connect(pipe.C_out, inputflowmodel.C_in) annotation (Line(points={{0,52},{0,48}},                                                         color={85,170,255}));
  connect(hot_side_cooling.C_out, Water_inlet.C_in) annotation (Line(points={{-50,0},{-27,0}},                                color={28,108,200}));
  connect(Water_outlet.C_out, C_hot_out) annotation (Line(points={{25,0},{90,0}}, color={28,108,200}));
  connect(C_cold_in, C_cold_in) annotation (Line(points={{0,90},{0,90}}, color={85,170,255}));
  connect(C_hot_out, C_hot_out) annotation (Line(points={{90,0},{90,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}), graphics={
        Ellipse(
          extent={{-20,110},{20,70}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Line(points={{-80,-80},{82,-80},{40,60},{-40,60},{-80,-80}}, color={28,108,200}),
        Ellipse(
          extent={{-48,82},{-40,74}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{32,114},{40,106}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{28,78},{36,70}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{-36,110},{-28,104}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{26,-44},{-28,22}},
          lineColor={28,108,200},
          fillColor={85,255,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})));
end CoolingTowerPoppe;
