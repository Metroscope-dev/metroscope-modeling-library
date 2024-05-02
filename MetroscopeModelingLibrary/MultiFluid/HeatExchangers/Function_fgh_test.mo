within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Function_fgh_test
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

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
    y:= ((Qw * cp) / Qa) * (1 + (((MoistAir.xsaturation_pT(Pin, Tw) - w) * (cp * Tw)) / ((MoistAir.h_pTX(Pin, Tw, {w}) - i + ((Lef-1) * (MoistAir.h_pTX(Pin, Tw, {w}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw))));
  end g;

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
    y:= ((cp * (Qw / Qa) * (MoistAir.xsaturation_pT(Pin, Tw) - w))) / (((MoistAir.h_pTX(Pin, Tw, {w})) - i + (Lef-1) * ((MoistAir.h_pTX(Pin, Tw, {w}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw));
  end f;

  function h
    input Real Tw;
    input Real w;
    input Real i;
    input Real cp;
    input Real Pin;
    input Real Lef;
    output Real y;
  algorithm
    y:= cp / (MoistAir.h_pTX(Pin, Tw, {w}) - i + (Lef-1) * (MoistAir.h_pTX(Pin, Tw, {w}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1})) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw);
  end h;

    // Poppe Inputs

  parameter Real rh = 0.8;
  parameter Real i = 17719.107;
  parameter Real Tw = 20+273.15;
  parameter Real cp = 4180;
  parameter Real Pin = 100000;
  parameter Real hd = 0.012856079;                       //From Merkel model
  Real Lef;
  parameter Real Qw = 38853.242;
  parameter Real Qa = 18717.115;

  Real w;
  Real w1;
  Real w2;
  Real w3;
  Real w4;

  Real i1;
  Real i2;
  Real i3;
  Real i4;

  Real M1;
  Real M2;
  Real M3;
  Real M4;
  Real Mtest;
  Real Mtest1;
  Real Mtest3;

  Real Qw1;
  Real Qw2;
  Real Qw3;

  Real Qa1;
  Real Qa2;
  Real Qa3;

  Real imasw;
  Real wsw;
  Real iv;
  Real P1;
  Real P2;
  Real P3;
  Real Tot;

equation

  // connectors

  // New Poppe Equations

  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622))-1) / log((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622));                                  //NEW

  //i - has a given start value, taken from Merkel model
  i1 = i + 2 * g(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  i2 = i1 + 2 * g(Tw-5, w1, i1, cp, Qw1, Qa1, Pin, Lef);
  i3 = i2 + 2 * g(Tw-10, w2, i2, cp, Qw2, Qa2, Pin, Lef);
  i4 = i3 + 2 * g(Tw-15, w3, i3, cp, Qw3, Qa3, Pin, Lef);

  w = MoistAir.massFraction_pTphi(Pin, Tw, rh);             //start value with relative humidity equal to 0.8
  w1 = w + 2 * f(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  w2 = w1 + 2 * f(Tw-5, w1, i1, cp, Qw1, Qa1, Pin, Lef);
  w3 = w2 + 2 * f(Tw-10, w2, i2, cp, Qw2, Qa2, Pin, Lef);
  w4 = w3 + 2 * f(Tw-15, w3, i3, cp, Qw3, Qa3, Pin, Lef);

  M1 = 1;                                                   //Possibly a standard merkel number start value - range of 0.5-2 online seems acceptable
  M2 = M1 + 2 * h(Tw-5, w1, i1, cp, Pin, Lef);
  M3 = M2 + 2 * h(Tw-10, w2, i2, cp, Pin, Lef);             //Merkel number is decreasing and can go negative depending on hd (and hence M1) and also depending on deltaTw
  M4 = M3 + 2 * h(Tw-15, w3, i3, cp, Pin, Lef);             //Negative Merkel number isn't supposed to happen
  Mtest = h(Tw, w, i, cp, Pin, Lef);

  Mtest1 = hd * 3000 / Qw;
  Mtest3 = hd * 3000 / Qw3;

  //Qw - has a given start value, taken from Merkel model
  Qw1 = Qw - Qa * (w1 - w);
  Qw2 = Qw1 - Qa1 * (w2 - w1);
  Qw3 = Qw2 - Qa2 * (w3 - w2);                              //Depending on Tw, Qw and Qa can both increase ?

  //Qa - has a given start value, taken from Merkel model
  Qa1 = Qa * (1 + w1);
  Qa2 = Qa1 * (1 + w2);
  Qa3 = Qa2 * (1 + w3);

  imasw = MoistAir.h_pTX(Pin, Tw, {w});
  wsw = MoistAir.xsaturation_pT(Pin, Tw);
  iv = MoistAir.h_pTX(Pin, Tw, {1});
  P1 = MoistAir.h_pTX(Pin, Tw, {w}) - i;
  P2 = (Lef - 1) * (MoistAir.h_pTX(Pin, Tw, {w}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}));
  P3 = -(MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw;
  Tot = cp / (P1 + P2 + P3);

  //di/dTw -  function g

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
end Function_fgh_test;
