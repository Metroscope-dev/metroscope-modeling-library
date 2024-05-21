within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTowerPoppe
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
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= (cp * (Qw / Qa) * (MoistAir.xsaturation_pT(Pin, Tw) - w)) / (((MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)})) - i + (Lef-1) * ((MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw));
  end f;

  function g
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real Qw;
    input Real Qa;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= ((Qw * cp) / Qa) * (1 + (((MoistAir.xsaturation_pT(Pin, Tw) - w) * (cp * Tw)) / ((MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i + ((Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw))));
  end g;

  function h
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= cp / (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1})) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw);
  end h;

  Units.Velocity V_inlet;
  Inputs.InputReal hd;
  Inputs.InputArea Afr;
  Inputs.InputReal Lfi;
  Inputs.InputFrictionCoefficient Cf;

  constant Real gr(unit="m/s2") = Modelica.Constants.g_n;

  Units.Density rho_air_inlet;
  Units.Density rho_air_outlet;

  Units.MassFlowRate Q_hot_in;
  Units.MassFlowRate Q_hot_out;
  Units.MassFlowRate Q_cold_in;
  Units.MassFlowRate Q_cold_out;

  Real w_in;
  Real w_out;

  Units.SpecificEnthalpy i_initial;
  Units.SpecificEnthalpy i_final;

  Units.Temperature T_cold_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

    // Poppe Inputs
  Units.Temperature deltaTw;

  parameter Integer N_step = 10;
  Real w[N_step];
  Real M[N_step];
  Real Me;
  Real i[N_step];
  Real Tw[N_step];
  //Real Ta[N_step];                                 //NEW
  Units.HeatCapacity cp[N_step];
  Real Pin[N_step];
  Real Lef[N_step];
  Units.MassFlowRate Qw[N_step];
  Units.MassFlowRate Qa[N_step];

  WaterSteam.Connectors.Inlet water_inlet_connector annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  WaterSteam.Connectors.Outlet water_outlet_connector annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet air_inlet_connector annotation (Placement(transformation(extent={{-10,100},{10,120}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet air_outlet_connector annotation (Placement(transformation(extent={{-10,-120},{10,-100}})));
  WaterSteam.BaseClasses.IsoHFlowModel  water_inlet_flow annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  WaterSteam.BaseClasses.IsoPHFlowModel water_outlet_flow annotation (Placement(transformation(extent={{64,-10},{84,10}})));
  WaterSteam.BoundaryConditions.Source water_outlet annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  WaterSteam.BoundaryConditions.Sink water_inlet annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel air_inlet_flow annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,56})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink   air_inlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,28})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source
                                                             air_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-32})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel air_outlet_flow annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-64})));
equation

  // connectors
  air_inlet_flow.P_out = Pin[1];
  air_inlet_flow.Q = Q_cold_in;
  air_inlet_flow.h = i_initial;
  air_inlet.T_in = T_cold_in;
  w_in = air_inlet.Xi_in[1];

  air_outlet_flow.P_in = Pin[N_step];
  air_outlet_flow.Q = Q_cold_out;
  air_outlet_flow.h = i_final;
  air_outlet.T_out = T_cold_out;
  w_out = air_outlet.Xi_out[1];

  water_inlet_flow.P_out = Pin[N_step];
  water_inlet_flow.Q = Q_hot_in;
  water_inlet_flow.T_in = T_hot_in;

  water_outlet_flow.P_out = Pin[1];
  water_outlet_flow.Q = Q_hot_out;
  water_outlet_flow.T_in = T_hot_out;

  // New Poppe Equations

  deltaTw = (Tw[N_step] - Tw[1]) / (N_step - 1);

  for n in 1:N_step loop
    Tw[n] = T_hot_out + (T_hot_in-T_hot_out)*(n-1)/(N_step-1);
  end for;

  for n in 1:N_step-1 loop
     w[n+1] = w[n] + deltaTw * f(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     i[n+1] = i[n] + deltaTw * g(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     M[n+1]= M[n] + deltaTw * h(Tw[n+1], w[n+1], i[n+1], cp[n+1], Pin[n+1], Lef[n+1]);

    Qw[n+1] = Qw[n] + Qa[n] * (w[n+1] - w[n]);

    Qa[n+1] = Qa[n] * (1 + (w[n+1] - w[n]));

    Lef[n+1] = Lef[n];

    cp[n+1] = cp[n];

    Pin[n+1] = Pin[n];

  end for;

  Me = hd * Afr / Qw[1];
  M[N_step] = Me;
  M[1] = 0;

  w[1] = w_in;
  w[N_step] = w_out;

  i[1] = i_initial;
  i[N_step] = i_final;

  Qw[1] = Q_hot_out;
  Qw[N_step] = Q_hot_in;
  Qa[1] = Q_cold_in;
  Qa[N_step] = Q_cold_out;

  Lef[1] = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622))-1) / log((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622));
  cp[1] = WaterSteamMedium.specificHeatCapacityCp(water_inlet_flow.state_in);

   // Drift Equation
   rho_air_inlet = air_inlet_flow.rho_in;
   rho_air_outlet = air_outlet_flow.rho_out;

   0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  =  (rho_air_inlet - rho_air_outlet) * gr * Lfi;
   Q_cold_in = (V_inlet * Afr * rho_air_inlet)* (1 - air_inlet.Xi_in[1]);

  connect(water_inlet_connector, water_inlet_flow.C_in) annotation (Line(points={{-110,0},{-76,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_out, water_outlet_connector) annotation (Line(points={{84,0},{110,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_in, water_outlet.C_out) annotation (Line(points={{64,0},{37,0}}, color={28,108,200}));
  connect(water_inlet_flow.C_out, water_inlet.C_in) annotation (Line(points={{-56,0},{-37,0}}, color={28,108,200}));
  connect(air_inlet_flow.C_in, air_inlet_connector) annotation (Line(points={{0,66},{0,110}}, color={85,170,255}));
  connect(air_outlet_flow.C_out, air_outlet_connector) annotation (Line(points={{0,-74},{0,-110}}, color={85,170,255}));
  connect(air_inlet_flow.C_out, air_inlet.C_in) annotation (Line(points={{0,46},{0,33}}, color={85,170,255}));
  connect(air_outlet_flow.C_in, air_outlet.C_out) annotation (Line(points={{0,-54},{0,-37}}, color={85,170,255}));
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
