within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Functiongtest
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

    // Poppe Inputs

  parameter Real rh = 0.8;
  parameter Real i = 87145.414;
  parameter Real Tw = 45+273.15;
  parameter Real cp = 4180;
  parameter Real Pin = 100000;
  Real Lef;
  parameter Real Qw = 38853.242;
  parameter Real Qa = 18717.115;
  Real y;
  Real w;
  Real i1;
  Real i2;
  Real i3;
  Real i4;

equation

  // connectors

  // New Poppe Equations

  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622))-1) / log((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622));                                  //NEW
  y = g(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  i1 = i + 10 * g(Tw-35*time, w+0.05*time, i+130000*time, cp, Qw-1000*time, Qa+3000*time, Pin, Lef);
  //i1 = i + 10 * g(Tw, w, i, cp, Qw, Qa, Pin, Lef);
  i2 = i1 + 10 * g(Tw, w, i1, cp, Qw, Qa, Pin, Lef);
  i3 = i2 + 10 * g(Tw, w, i2, cp, Qw, Qa, Pin, Lef);
  i4 = i3 + 10 * g(Tw, w, i3, cp, Qw, Qa, Pin, Lef);

  w = MoistAir.massFraction_pTphi(Pin, Tw, rh);

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
end Functiongtest;
