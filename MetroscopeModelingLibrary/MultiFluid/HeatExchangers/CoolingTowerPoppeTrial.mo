within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTowerPoppeTrial
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
    y:= ((cp * (Qw / Qa) * (MoistAir.xsaturation_pT(Pin, Tw) - w))) / (((MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)})) - i + (Lef-1) * ((MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(Pin, 100+273.15, 1))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw));
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
    y:= ((Qw * cp) / Qa) * (1 + (((MoistAir.xsaturation_pT(Pin, Tw) - w) * (cp * Tw)) / ((MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)}) - i + ((Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(Pin, 100+273.15, 1))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw))));
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
    y:= cp / (MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, 1)}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * WaterSteamMedium.specificEnthalpy_pT(Pin, 100+273.15, 1)) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw);
  end h;


  Inputs.InputReal hd;

  Units.MassFlowRate Qw[N_step];
  Units.MassFlowRate Qa[N_step];
  Units.MassFlowRate Q_hot_in;
  Units.MassFlowRate Q_hot_out;
  Units.MassFlowRate Q_cold_in;
  Units.MassFlowRate Q_cold_out;

  Inputs.InputReal w_in;
  Inputs.InputReal w_out;

  Units.SpecificEnthalpy i_initial;
  Units.SpecificEnthalpy i_final;

  Units.HeatCapacity cp[N_step];                       //NEW


  Units.Temperature T_cold_in;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

    // Poppe Inputs
  Units.Temperature deltaT;

  parameter Integer N_step = 10;
  Real w[N_step];
  Real M[N_step];
  Real i[N_step];
  Real Tw[N_step];

  Real Pin[N_step];                                   //NEW
  Real Lef[N_step];                                   //NEW

  WaterSteam.Connectors.Inlet water_inlet_connector annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  WaterSteam.Connectors.Outlet water_outlet_connector annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet air_inlet_connector annotation (Placement(transformation(extent={{-10,100},{10,120}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet air_outlet_connector annotation (Placement(transformation(extent={{-10,-120},{10,-100}})));
  WaterSteam.BaseClasses.IsoHFlowModel  water_inlet_flow annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  WaterSteam.BaseClasses.IsoPHFlowModel water_outlet_flow annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  WaterSteam.BoundaryConditions.Source water_outlet annotation (Placement(transformation(extent={{34,-10},{54,10}})));
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
        origin={0,-36})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel air_outlet_flow annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-64})));
equation

  // connectors

  air_inlet_connector.P = Pin[1];
  air_inlet_connector.Q = Q_cold_in;
  air_inlet_connector.h_outflow = i_initial;
  w[1] = air_inlet.relative_humidity * MoistAir.xsaturation(air_inlet_flow.state_in);
  air_inlet.T_in = T_cold_in;


  air_outlet_connector.P = Pin[1];
  air_outlet_connector.Q = Q_cold_out;
  air_outlet_connector.h_outflow = i_final;
  w[N_step] = air_outlet.relative_humidity * MoistAir.xsaturation(air_outlet_flow.state_out);


  water_inlet_flow.P_out = Pin[1];
  water_inlet_flow.Q = Q_hot_in;
  water_inlet_flow.T_in = Tw[1];
  cp[1] = WaterSteamMedium.specificHeatCapacityCp(water_inlet_flow.state_in);

  water_outlet_flow.P_out = Pin[1];
  water_outlet_flow.Q = Q_hot_out;
  water_outlet_flow.T_in = Tw[N_step];



  // New Poppe Equations

  Lef[1] = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622))-1) / log((MoistAir.xsaturation_pT(Pin[1], T_cold_in)+0.622)/(w[1]+0.622));

  deltaT = (Tw[1] - Tw[N_step])/N_step;

  for n in 1:N_step-1 loop
    w[n+1] = w[n] + deltaT * f(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);
    i[n+1] = i[n] + deltaT * g(Tw[n], w[n], i[n], cp[n], Qw[n], Qa[n], Pin[n], Lef[n]);         //MAKE ALL OD THESE A FUNCTION OF N
    M[n+1]= M[n] + deltaT * h(Tw[n], w[n], i[n], cp[n], Pin[n], Lef[n]);
    Qw[n+1] = Qw[n] - Qa[n] * (w[n+1] - w[n]);
    Qa[n+1] = Qa[n] * (1 + w[n+1]);
    M[n+1] = hd / Qw[n+1];

    Lef[n+1] = Lef[n];                                        //NEW
    Pin[n+1] = Pin[n];                                        //NEW
    Tw[n+1] = Tw[n] - deltaT;                        //NEW
    cp[n+1] = cp[n];                                          //NEW
  end for;

  w[1] = w_in;
  w[N_step] = w_out;
  i[1] = i_initial;
  i[N_step] = i_final;
  Tw[1] = T_hot_in "degC";
  Tw[N_step] = T_hot_out "degC";
  M[1] = hd / Qw[1];
  //M[N_step] = hd / Qw[N_step];                                   //NEW
  Qw[1] = Q_hot_in;
  Qw[N_step] = Q_hot_out;
  Qa[1] = Q_cold_in;
  Qa[N_step] = Q_cold_out;
  //Pin[1] = Pin[N_step];                                          //NEW
  //Lef[1] = Lef[N_step];                                          //NEW
  //cp[1] = cp[N_step];                                            //NEW

  connect(water_inlet_connector, water_inlet_flow.C_in) annotation (Line(points={{-110,0},{-80,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_out, water_outlet_connector) annotation (Line(points={{80,0},{110,0}}, color={28,108,200}));
  connect(water_outlet_flow.C_in, water_outlet.C_out) annotation (Line(points={{60,0},{49,0}}, color={28,108,200}));
  connect(water_inlet_flow.C_out, water_inlet.C_in) annotation (Line(points={{-60,0},{-37,0}}, color={28,108,200}));
  connect(air_inlet_flow.C_in, air_inlet_connector) annotation (Line(points={{0,66},{0,110}}, color={85,170,255}));
  connect(air_outlet_flow.C_out, air_outlet_connector) annotation (Line(points={{0,-74},{0,-110}}, color={85,170,255}));
  connect(air_inlet_flow.C_out, air_inlet.C_in) annotation (Line(points={{0,46},{0,33}}, color={85,170,255}));
  connect(air_outlet_flow.C_in, air_outlet.C_out) annotation (Line(points={{0,-54},{0,-41}}, color={85,170,255}));
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
end CoolingTowerPoppeTrial;
