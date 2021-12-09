within MetroscopeModelingLibrary.FlueGases.Machines;
model CombustionChamber
   package FuelMedium =
      MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
   package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel(P_in(start=1e5), P_out(start=45e5),h_in(start=1e5), h_out(start=1.2e5),redeclare package
              Medium =FlueGasesMedium);
  Modelica.Units.SI.Power Wth(start=850e6);
  Modelica.Units.SI.Power Wfuel(start=800e6);
  Real LHV(start=48130);
  Modelica.Units.SI.MassFlowRate Qfuel(start=-15);
  constant Real amC=12.01115 "Carbon atomic mass";
  constant Real amH=1.00797 "Hydrogen atomic mass";
  constant Real amO=15.9994 "Oxygen atomic mass";
  Real amCH4 "CH4 molecular mass";
  Real amC2H6 "C2H6 molecular mass";
  Real amC3H8 "C3H8 molecular mass";
  Real amC4H10 "C4H10 molecular mass";
  Real amCO2 "CO2 molecular mass";
  Real amH2O "H2O molecular mass";
  Real Kfr(start=1.e3) "Pressure loss coefficient";
  FlueGasesMedium.MassFraction X_in_N2(start=0.78);
  FlueGasesMedium.MassFraction X_in_O2(start=0.22);
  FlueGasesMedium.MassFraction X_in_H2O(start=0.0);
  FlueGasesMedium.MassFraction X_in_CO2(start=0.0);
  FlueGasesMedium.MassFraction X_in_SO2(start=0.0);
  FlueGasesMedium.MassFraction X_out_N2(start=0.73);
  FlueGasesMedium.MassFraction X_out_O2(start=0.12);
  FlueGasesMedium.MassFraction X_out_H2O(start=0.10);
  FlueGasesMedium.MassFraction X_out_CO2(start=0.05);
  FlueGasesMedium.MassFraction X_out_SO2(start=0.0);
  FlueGasesMedium.MassFraction X_fuel_CH4(start=0.92);
  FlueGasesMedium.MassFraction X_fuel_C2H6(start=0.048);
  FlueGasesMedium.MassFraction X_fuel_C3H8(start=0.005);
  FlueGasesMedium.MassFraction X_fuel_C4H10_n_butane(start=0.002);
  FlueGasesMedium.MassFraction X_fuel_N2(start=0.015);
  FlueGasesMedium.MassFraction X_fuel_CO2(start=0.01);
  Real X_fuel_C(start=0.8) "C mass fraction in the fuel";
  Real X_fuel_H(start=0.2) "H mass fraction in the fuel";
  Real X_fuel_O(start=0) "O mass fraction in the fuel";
  Fuel.Connectors.FluidInlet   C_fuel
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
equation

  /* Mass Balance */
  Q_in  + Qfuel + Q_out= 0;
  Qfuel = C_fuel.Q;

  /* Mechanical Balance */
  P_in - P_out = Kfr*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_in, eps)/rhom;

  /* Thermal Power */
  Wth = -(Q_out*h_out + Q_in*h_in + C_fuel.H);
  Wfuel = LHV*Qfuel;
  Wth = Wfuel;
  C_fuel.h_vol = 1e6; //definition of C_fuel.h_vol needed since the connector is not connected to a basic transport model

  /*Chemical balance */

  // Initial quantities
  C_fuel.Xi_vol = FuelMedium.X_default;
  X_in_N2 = Xi_in[1];
  X_in_O2 = Xi_in[2];
  X_in_H2O = Xi_in[3];
  X_in_CO2 = Xi_in[4];
  X_in_SO2 = Xi_in[5];
  Qfuel*X_fuel_CH4 = C_fuel.Qi[1]; // methane
  Qfuel*X_fuel_C2H6 = C_fuel.Qi[2]; // ethane
  Qfuel*X_fuel_C3H8=  C_fuel.Qi[3]; // propane
  Qfuel*X_fuel_C4H10_n_butane=  C_fuel.Qi[4]; //butane
  Qfuel*X_fuel_N2=  C_fuel.Qi[5]; // nitrogen
  Qfuel*X_fuel_CO2=  C_fuel.Qi[6]; // carbon dioxyde

  // Final quantities
  X_out_N2 = Xi_out[1];
  X_out_O2 = Xi_out[2];
  X_out_H2O = Xi_out[3];
  X_out_CO2 = Xi_out[4];
  X_out_SO2 = Xi_out[5];

  // Atomic mass of species of interest
  amCH4 = amC + 4*amH;
  amC2H6 = 2*amC + 6*amH;
  amC3H8 = 3*amC + 8*amH;
  amC4H10 = 4*amC + 10*amH;
  amCO2 = amC + 2*amO;
  amH2O = 2*amH + amO;

  // quantity of reactants in fuel
  Qfuel*X_fuel_C  = amC* (Qfuel*X_fuel_CH4/amCH4 + 2*Qfuel*X_fuel_C2H6/amC2H6 + 3*Qfuel*X_fuel_C3H8/amC3H8 + 4*Qfuel*X_fuel_C4H10_n_butane/amC4H10 + Qfuel*X_fuel_CO2/amCO2);
  Qfuel*X_fuel_H  = amH*(4*Qfuel*X_fuel_CH4/amCH4 + 6*Qfuel*X_fuel_C2H6/amC2H6 + 8*Qfuel*X_fuel_C3H8/amC3H8 + 10*Qfuel*X_fuel_C4H10_n_butane/amC4H10);
  Qfuel*X_fuel_O = 2*amO*Qfuel*X_fuel_CO2/amCO2;

  /* Mass balance for all species */
  Q_out * X_out_N2  + Q_in * X_in_N2  + Qfuel*X_fuel_N2= 0; //Hyp: the NOx creation is negligible
  Q_out * X_out_O2  + Q_in * X_in_O2  + Qfuel*X_fuel_O= Qfuel*amO*(2*X_fuel_C/amC + 0.5*X_fuel_H/amH);
  Q_out * X_out_H2O + Q_in * X_in_H2O = -Qfuel*(0.5*X_fuel_H/amH)*amH2O;
  Q_out * X_out_CO2 + Q_in * X_in_CO2 = -Qfuel*X_fuel_C*amCO2/amC;
  Q_out * X_out_SO2 + Q_in * X_in_SO2 = 0; //Hyp: No S in fuel


  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{100,100},{100,-100},{-100,-100},{-100,100},{100,100}},
          lineColor={0,0,255},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-32,62},{34,46},{-10,30},{46,18},{-6,0},{36,-16},{-10,-32},{42,
              -44},{-32,-60},{-32,62}},
          lineColor={0,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.CrossDiag)}),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end CombustionChamber;
