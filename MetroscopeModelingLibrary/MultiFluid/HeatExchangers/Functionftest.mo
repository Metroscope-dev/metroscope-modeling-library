within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Functionftest
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
    y:= ((cp * (Qw / Qa) * (MoistAir.xsaturation_pT(Pin, Tw) - w))) / (((MoistAir.h_pTX(Pin, Tw, {w})) - i + (Lef-1) * ((MoistAir.h_pTX(Pin, Tw, {w}) - i - (MoistAir.xsaturation_pT(Pin, Tw) - w) * MoistAir.h_pTX(Pin, Tw, {1}))) - (MoistAir.xsaturation_pT(Pin, Tw) - w) * cp * Tw));
  end f;


    // Poppe Inputs

  parameter Real rh = 1;
  parameter Real i = 17719.107;
  parameter Real Tw = 45+273.15;
  parameter Real cp = 4180;
  parameter Real Pin = 100000;
  Real Lef;
  parameter Real Qw = 38853.242;
  parameter Real Qa = 18717.115;

  Real w;
  Real w2;
  Real w3;
  Real w4;
  Real y;
  Real y2;
  Real abshumid;
  Real enthalp;
  Real steam;
  Real deltaw;
  Real Qw1;
  Real Qw2;
  Real Qw3;
  Real Qa1;
  Real Qa2;
  Real Qa3;
  Real Qa11;
  Real Qa22;
  Real Qa33;

equation

  // connectors

  w = MoistAir.massFraction_pTphi(Pin, Tw, rh);

  // New Poppe Equations}
  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622))-1) / log((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622));                                  //NEW
  y = f(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  y2 = f(Tw, w+0.5*time, i, cp, Qw, Qa, Pin, Lef);

  w2 = w + 10 * f(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  w3 = w2 + 10 * f(Tw, w2, i, cp, Qw2, Qa2, Pin, Lef);
  w4 = w3 + 10 * f(Tw, w3, i, cp, Qw3, Qa3, Pin, Lef);

  abshumid = MoistAir.xsaturation_pT(Pin, Tw);
  deltaw = MoistAir.xsaturation_pT(Pin, Tw) - w;
  enthalp = MoistAir.h_pTX(Pin, Tw, {MoistAir.massFraction_pTphi(Pin, Tw, w)}) - i;
  steam = WaterSteamMedium.specificEnthalpy_pT(Pin, 100.05+273.15, 0);

  Qw1 = Qw - Qa * (w2 - w);
  Qw2 = Qw1 - Qa1 * (w3 - w2);
  Qw3 = Qw2 - Qa2 * (w4 - w3);

  Qa1 = Qa * (1 + w);
  Qa2 = Qa1 * (1 + w2);
  Qa3 = Qa2 * (1 + w3);

  Qa11 = Qa + Qa * (w2 - w);
  Qa22 = Qa1 + Qa1 * (w3 - w2);
  Qa33 = Qa2 + Qa2 * (w4 - w3);



  //dw/dTw -  humidity ratio/water temperature - function f

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
end Functionftest;
