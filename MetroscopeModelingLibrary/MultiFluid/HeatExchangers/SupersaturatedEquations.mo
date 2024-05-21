within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model SupersaturatedEquations
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

    function j
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

    function k
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

    function m
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



    // Poppe Inputs

  parameter Real rh = 1.1;

  parameter Real Tw_in = 30+273.15;
  input Real Tw_out(start = 20+273.15);

  parameter Real Ta_in = 15+273.15;
  input Real Ta_out(start= 30+273.15);

  Real Tw[4];
  Real deltaTw;

  Real Ta[4];
  Real deltaTa;


  parameter Real cp = 4180;
  parameter Real Pin = 100000;
  output Real hd(start= -0.0012223658);                       //From Merkel model
  Real Lef;

  Real w1;
  Real w2;
  Real w3;
  Real w4;

  Real rh4;

  Real i1;
  Real i2;
  Real i3;
  Real i4;
  Real Ta4;

  Real M1;
  Real M2;
  Real M3;
  Real M4;
  Real Me;

  Real Qw1;
  Real Qw2;
  Real Qw3;
  parameter Real Qw4 = 30;

  parameter Real Qa1 = 20;
  Real Qa2;
  Real Qa3;
  Real Qa4;

equation
   // Water temperature
   Tw[1] = Tw_out;
   Tw[2] = Tw[1] + deltaTw;
   Tw[3] = Tw[2] + deltaTw;
   Tw[4] = Tw_in;
   deltaTw = (Tw[4] - Tw[1]) / (4 - 1);

   // Air Temperature
   Ta[1] = Ta_in;
   Ta[2] = Ta[1] + deltaTa;
   Ta[3] = Ta[2] + deltaTa;
   Ta[4] = Ta_out;
   deltaTa = (Ta[4] - Ta[1])/ (4 - 1);

  // Lewis factor
  Lef = 0.9077990913 * (((MoistAir.xsaturation_pT(Pin, Ta_in)+0.622)/(w1+0.622))-1) / log((MoistAir.xsaturation_pT(Pin, Ta_in)+0.622)/(w1+0.622));                                  //NEW

  // Air enthalpy
  i1 = MoistAir.h_pTX(Pin, Ta_in, {w1});                                  //inlet
  i2 = i1 + deltaTw * k(Tw[1], Ta[2], w1, i1, cp, Qw1, Qa1, Pin, Lef);
  i3 = i2 + deltaTw * k(Tw[2], Ta[3], w2, i2, cp, Qw2, Qa2, Pin, Lef);
  i4 = i3 + deltaTw * k(Tw[3], Ta[4], w3, i3, cp, Qw3, Qa3, Pin, Lef);         //outlet

  Ta4 = MoistAir.T_phX(Pin, i4, {w4});

  // Humidity
  w1 = MoistAir.massFraction_pTphi(Pin, Ta_in, rh);                       //inlet
  w2 = w1 + deltaTw * j(Tw[1], Ta[2], w1, i1, cp, Qw1, Qa1, Pin, Lef);
  w3 = w2 + deltaTw * j(Tw[2], Ta[3], w2, i2, cp, Qw2, Qa2, Pin, Lef);
  w4 = w3 + deltaTw * j(Tw[3], Ta[4], w3, i3, cp, Qw3, Qa3, Pin, Lef);         //outlet
  w4 = MoistAir.massFraction_pTphi(Pin, Tw[4], rh4);

  // Merkel number
  M1 =  deltaTw * m(Tw[1], Ta[1], w1, i1, cp, Pin, Lef);
  M2 =  deltaTw * m(Tw[2], Ta[2], w2, i2, cp, Pin, Lef);             //Merkel number is decreasing and can go negative depending on hd (and hence M1) and also depending on deltaTw
  M3 =  deltaTw * m(Tw[3], Ta[3], w3, i3, cp, Pin, Lef);             //Negative Merkel number isn't supposed to happen
  M4 =  deltaTw * m(Tw[4], Ta[4], w4, i4, cp, Pin, Lef);
  Me = (M1 + M2 + M3 + M4) / (4 * deltaTw);
  Me = hd * 3000 / Qw1;


  // Water flow
  //Qw - has a given start value, taken from Merkel model
  Qw2 = Qw1 - Qa1 * (w2 - w1);
  Qw3 = Qw2 - Qa2 * (w3 - w2);
  Qw4 = Qw3 - Qa3 * (w4 - w3);

  // Air flow
  //Qa - has a given start value, taken from Merkel model
  Qa2 = Qa1 * (1 + w2);
  Qa3 = Qa2 * (1 + w3);
  Qa4 = Qa3 * (1 + w4);



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
end SupersaturatedEquations;
