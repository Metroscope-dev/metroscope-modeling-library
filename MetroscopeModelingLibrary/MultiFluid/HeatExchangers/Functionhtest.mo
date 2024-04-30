within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Functionhtest
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

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
  parameter Real Tw = 40+273.15;
  parameter Real cp = 4180;
  parameter Real Pin = 100000;
  Real Lef;
  parameter Real Qw = 38853.242;
  parameter Real Qa = 18717.115;
  Real y;
  Real hum;
  Real w;
  Real M1;
  Real M2;
  Real M3;
  Real M4;
equation

  // connectors

  // New Poppe Equations

  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622))-1) / log((MoistAir.xsaturation_pT(Pin, 5+273.15)+0.622)/(w+0.622));                                  //NEW
  y = h(Tw, w, i, cp, Pin, Lef);
  hum = MoistAir.xsaturation_pT(Pin, Tw);
  w = MoistAir.massFraction_pTphi(Pin, Tw, rh);

  M1 = 0.000980808;
  M2 = M1 + 10 * h(Tw-5, w+0.02, i+10000, cp, Pin, Lef);
  M3 = M2 + 10 * h(Tw-10, w+0.035, i+20000, cp, Pin, Lef);
  M4 = M3 + 10 * h(Tw-25, w+0.04, i+160000, cp, Pin, Lef);


  //dMe/dTw = function h

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
end Functionhtest;
