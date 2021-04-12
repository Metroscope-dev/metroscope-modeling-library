within MetroscopeModelingLibrary.FlueGases.Machines;
model CombustionChamber_LHV
  extends CombustionChamber;
  constant  Real efmCH4= -74.5e3 "CH4 molar enthalpy of formation";
  constant  Real efmC2H6= -83.8e3 "C2H6 molar enthalpy of formation";
  constant  Real efmC3H8=-104.7e3 "C3H8 molar enthalpy of formation";
  constant  Real efmC4H10= -125.6e3 "C4H10 molar enthalpy of formation";
  constant  Real efmCO2= -393.5e3 "CO2 molar enthalpy of formation";
  constant  Real efmH2O= -241.8e3 "H2O molar enthalpy of formation";
  Real efCH4 "CH4  enthalpy of formation";
  Real efC2H6 "C2H6  enthalpy of formation";
  Real efC3H8 "C3H8  enthalpy of formation";
  Real efC4H10 "C4H10  enthalpy of formation";
  Real efCO2 "CO2  enthalpy of formation";
  Real efH2O "H2O  enthalpy of formation";

equation
   efCH4= 1e3*efmCH4/amCH4 "CH4 specific enthalpy of formation";
   efC2H6= 1e3*efmC2H6/amC2H6 "C2H6 specific enthalpy of formation";
   efC3H8= 1e3*efmC3H8/amC3H8 "C3H8 specific enthalpy of formation";
   efC4H10= 1e3*efmC4H10/amC4H10 "C4H10 specific enthalpy of formation";
   efCO2= 1e3*efmCO2/amCO2 "CO2 specific enthalpy of formation";
   efH2O= 1e3*efmH2O/amH2O "H2O specific enthalpy of formation";
   LHV=-(-(X_out_H2O*Q_out/Qfuel+X_in_H2O*Q_in/Qfuel)*efH2O-(X_out_CO2*Q_out/Qfuel+X_in_CO2*Q_in/Qfuel+X_fuel_CO2)*efCO2-X_fuel_CH4*efCH4-X_fuel_C2H6*efC2H6-X_fuel_C3H8*efC3H8-X_fuel_C4H10_n_butane*efC4H10);

end CombustionChamber_LHV;
