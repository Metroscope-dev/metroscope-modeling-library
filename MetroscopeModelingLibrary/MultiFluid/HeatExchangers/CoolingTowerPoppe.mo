within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTowerPoppe
                             //Reference Paper: J.C. Kloppers, D.G. Kröger, A critical investigation into the heat and mass transfer analysis of counterflow wet-cooling towers, International Journal of Heat and Mass Transfer, Volume 48, Issues 3–4, 2005, Pages 765-777, ISSN 0017-9310

  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

  parameter Integer N_step = 10; //Parameter specifies the number of sections for which the Cooling Tower thermodynamic properties are divided into in the loop.

  //Unsaturated Air - Equations from Poppe Method for Cooling Tower Analysis

  function f    "Differential function of absolute humidity with water temperature passing through Cooling Tower"
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

  function g    "Differential function of moist air enthalpy with water temperature passing through Cooling Tower"
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

  function h    "Differential function of Poppe Merkel number with water temperature passing through Cooling Tower"
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


  //Supersaturated Air - Equations from Poppe Method for Cooling Tower Analysis

  function j    "Differential function of absolute humidity with water temperature passing through Cooling Tower"
    input Real Tw;
    input Real Ta;
    input Real w;
    input Real i;
    input Real cp;
    input Real Qw;
    input Real Qa;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
      y:= (cp * (Qw / Qa) * (MoistAir.xsaturation_pT(Pin, Tw) - MoistAir.xsaturation_pT(Pin, Ta))) / (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - MoistAir.xsaturation_pT(Pin, Ta)) * (MoistAir.h_pTX(Pin, Tw, {1})) + (w - MoistAir.xsaturation_pT(Pin, Ta)) * cp * Tw) + (w - MoistAir.xsaturation_pT(Pin, Tw)) * cp * Tw);
  end j;

  function k    "Differential function of moist air enthalpy with water temperature passing through Cooling Tower"
    input Real Tw;
    input Real Ta;
    input Real w;
    input Real i;
    input Real cp;
    input Real Qw;
    input Real Qa;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= (cp * (Qw / Qa)) * (1 + (((cp * Tw) * (MoistAir.xsaturation_pT(Pin, Tw) - MoistAir.xsaturation_pT(Pin, Ta))) / (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - MoistAir.xsaturation_pT(Pin, Ta)) * MoistAir.h_pTX(Pin, Tw, {1}) + (w - MoistAir.xsaturation_pT(Pin, Ta)) * cp * Tw) + (w - MoistAir.xsaturation_pT(Pin, Tw)) * cp * Tw)));
  end k;

  function m    "Differential function of Poppe Merkel number with water temperature passing through Cooling Tower"
    input Real Tw;
    input Real Ta;
    input Real w;
    input Real i;
    input Real cp;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= cp / ((MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.xsaturation_pT(Pin, Tw)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - MoistAir.xsaturation_pT(Pin, Ta)) * MoistAir.h_pTX(Pin, Tw, {1})) + (w - MoistAir.xsaturation_pT(Pin, Ta)) * cp * Tw) + (w - MoistAir.xsaturation_pT(Pin, Tw)) * cp * Tw);
  end m;


  parameter String configuration = "natural";

  Units.Velocity V_inlet;                             //Air velocity at the bottom of the cooling tower
  Inputs.InputReal hd;                                //Mass transfer coefficient
  Inputs.InputArea Afr;                               //Tower cross-sectional area
  Inputs.InputReal Lfi;                               //Height of filling/packing
  Inputs.InputFrictionCoefficient Cf;                 //Friction coefficient of air
  Inputs.InputReal eta_fan;                           //Fan effiency
  Units.Power W_fan;                                  //Fan power

  constant Real gr(unit="m/s2") = Modelica.Constants.g_n;

  Units.Density rho_air_inlet;
  Units.Density rho_air_outlet;

  Units.MassFlowRate Q_hot_in;
  Units.MassFlowRate Q_hot_out;
  Units.MassFlowRate Q_cold_in;
  Units.MassFlowRate Q_cold_out;
  Units.MassFlowRate Q_evap;

  Real w_in;
  Real w_out;
  Real w_sat[N_step];

  Units.SpecificEnthalpy i_initial;
  Units.SpecificEnthalpy i_final;

  Units.Power W_max;
  Units.Power W_min;

  Units.Temperature T_cold_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

  Units.Temperature deltaTw;
  Units.Pressure deltaP_fan;                             //Pressure Change across Fan

  Real w[N_step];
  Real M[N_step];
  Real Me;
  Real i[N_step];
  Real Tw[N_step];
  Real Ta[N_step];
  Units.HeatCapacity cp[N_step];
  Real Pin[N_step];
  Real Lef[N_step];
  Units.MassFlowRate Qw[N_step];
  Units.MassFlowRate Qa[N_step];

  WaterSteam.Connectors.Inlet water_inlet_connector annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  WaterSteam.Connectors.Outlet water_outlet_connector annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet air_inlet_connector annotation (Placement(transformation(extent={{-10,-118},{10,-98}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet air_outlet_connector annotation (Placement(transformation(extent={{-10,98},{10,118}})));
  WaterSteam.BaseClasses.IsoHFlowModel  water_inlet_flow annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  WaterSteam.BaseClasses.IsoPHFlowModel water_outlet_flow annotation (Placement(transformation(extent={{64,-10},{84,10}})));
  WaterSteam.BoundaryConditions.Source water_outlet annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  WaterSteam.BoundaryConditions.Sink water_inlet annotation (Placement(transformation(extent={{-36,-10},{-16,10}})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel air_inlet_flow annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-74})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink   air_inlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-30})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source
                                                             air_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,24})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel air_outlet_flow annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,66})));
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

  W_max = Qw[10] * cp[1] * (Tw[N_step] - Tw[1]);
  W_min = Qw[1] * cp[1] * (Tw[N_step] - Tw[1]);

  Q_evap = (Q_cold_out - Q_cold_in);

  //New Poppe Equations

  deltaTw = (Tw[N_step] - Tw[1]) / (N_step - 1);

  for n in 1:N_step loop                                                                    //This loop causes the progression of the water and air temperatures and defines the saturation humidity based on N_step as they pass through the cooling tower
    Tw[n] = T_hot_out + (T_hot_in-T_hot_out)*(n-1)/(N_step-1);
    Ta[n] = MoistAir.T_phX(Pin[n], i[n], {w[n]});
    w_sat[n] = MoistAir.xsaturation_pT(Pin[n], Ta[n]);
  end for;

  for n in 1:N_step-1 loop                                                                  //This loop updates the value of thermodynamic variables, air/water flows, pressure, heat capacity and Merkel number using the governing equations as these media pass through the cooling tower

  if w[n] < w_sat[n] then                                                                   //This if condition switches the governing equations from the unsaturated to supersaturated ones, once the humidity in the tower exceeds the saturation humidity of the ambient conditions
     w[n+1] = w[n] + deltaTw * f(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     i[n+1] = i[n] + deltaTw * g(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     M[n+1]= M[n] + deltaTw * h(Tw[n+1], w[n+1], i[n+1], cp[n+1], Pin[n+1], Lef[n+1]);

    Qw[n+1] = Qw[n] + Qa[n] * (w[n+1] - w[n]);

    Qa[n+1] = Qa[n] * (1 + (w[n+1] - w[n]));

    Lef[n+1] = Lef[n];

    cp[n+1] = cp[n];

    Pin[n+1] = Pin[n];

   else
     w[n+1] = w[n] + deltaTw * j(Tw[n], Ta[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     i[n+1] = i[n] + deltaTw * k(Tw[n], Ta[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
     M[n+1]= M[n] + deltaTw * m(Tw[n+1], Ta[n+1], w[n+1], i[n+1], cp[n+1], Pin[n+1], Lef[n+1]);

    Qw[n+1] = Qw[n] + Qa[n] * (w[n+1] - w[n]);

    Qa[n+1] = Qa[n] * (1 + (w[n+1] - w[n]));

    Lef[n+1] = Lef[n];

    cp[n+1] = cp[n];

    Pin[n+1] = Pin[n];

  end if;

  end for;

  Me = hd * Afr / Qw[1];                //Can be Qw[N_step] but makes little difference
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

  Lef[1] = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622))-1) / log((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622));         //Can change w[1] to w[N_step] but little impact on Lef (-0.06 ish)
  cp[1] = WaterSteamMedium.specificHeatCapacityCp(water_inlet_flow.state_in);

   // Drift Equation
   rho_air_inlet = air_inlet_flow.rho_in;
   rho_air_outlet = air_outlet_flow.rho_out;

   deltaP_fan = (W_fan * eta_fan)/(abs(V_inlet) * Afr);

  if configuration == "natural" then

   0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  =  (rho_air_inlet - rho_air_outlet) * gr * Lfi;
   Q_cold_in = (V_inlet * Afr * rho_air_inlet)* (1 - air_inlet.Xi_in[1]);

  elseif configuration == "mechanical" then

  0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  = (W_fan * eta_fan)/(abs(V_inlet) * Afr);
  Q_cold_in = (V_inlet * Afr * rho_air_inlet * (1 - air_inlet.Xi_in[1]));

  end if;

  connect(water_inlet_connector, water_inlet_flow.C_in) annotation (Line(points={{-110,0},{-76,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_out, water_outlet_connector) annotation (Line(points={{84,0},{110,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_in, water_outlet.C_out) annotation (Line(points={{64,0},{37,0}}, color={28,108,200}));
  connect(water_inlet_flow.C_out, water_inlet.C_in) annotation (Line(points={{-56,0},{-31,0}}, color={28,108,200}));
  connect(air_inlet_flow.C_in, air_inlet_connector) annotation (Line(points={{0,-84},{0,-108}},
                                                                                              color={85,170,255}));
  connect(air_outlet_flow.C_out, air_outlet_connector) annotation (Line(points={{0,76},{0,108}},   color={85,170,255}));
  connect(air_inlet_flow.C_out, air_inlet.C_in) annotation (Line(points={{0,-64},{0,-35}},
                                                                                         color={85,170,255}));
  connect(air_outlet_flow.C_in, air_outlet.C_out) annotation (Line(points={{0,56},{0,29}},   color={85,170,255}));
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
