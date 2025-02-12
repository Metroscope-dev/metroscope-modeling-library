within MetroscopeModelingLibrary.Utilities;
package Constants "Stores all constants used in MML"
  extends Modelica.Icons.Package;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Gravity
  final constant Modelica.Units.SI.Acceleration g = Modelica.Constants.g_n;

  // Math constants
  final constant Real pi = Modelica.Constants.pi;

  // Temperature conversions
  final constant Units.Temperature T0_degC_in_K = 273.15;
  final constant Real T0_degC_in_degF(unit="degF") = 32;
  final constant Real degC_to_degF(unit="degF/degC") = 1.8;

  // Pressure conversions
  final constant Real Pa_to_barA(unit="bar/Pa") = 1e-5;
  final constant Real Pa_to_mbar(unit="mbar/Pa") = 1e-2;
  final constant Real Pa_to_psiA(unit="1/Pa") = 0.000145038;
  final constant Real Pa_to_inHg(unit="1/Pa") = 0.0002953006;
  final constant Real Pa_to_inH2O(unit="1/Pa") =0.0040146307866177;
  final constant Real Pa_to_kPaA(unit="1/Pa") = 0.001;
  final constant Real Pa_to_MPaA(unit="1/Pa") = 0.000001;
  final constant Real atmospheric_pressure_in_bar(unit="bar") = 1;
  final constant Real P0_psiG_in_psiA(unit="bar") = 14.50377377;
  final constant Real P0_kPaG_in_kPaA(unit="bar") = 100;
  final constant Real P0_MPaG_in_MPaA(unit="bar") = 0.1;

  // Volume flow conversions
  final constant Real m3s_to_lm = 60000;
  final constant Real m3s_to_ft3h= 127132.8002;
  // Mass flow conversions
  final constant Real kgs_to_th(unit="(1/h)/(kg/s)") = 3.6;
  final constant Real kgs_to_lbs(unit="1/kg") = 2.2046;
  final constant Real kgs_to_Mlbh(unit="(1/h)/(kg/s)") = 0.0079366414387;
  final constant Real kgs_to_lbh(unit="(1/h)/(kg/s)")= 7936.647912661459;

  // Atomic/Molecular masses
  final constant Units.AtomicMass m_H = 1.00798;
  final constant Units.AtomicMass m_H2 = m_H*2;
  final constant Units.AtomicMass m_C = 12.0106;
  final constant Units.AtomicMass m_O = 15.9994;
  final constant Units.MolecularMass m_CH4 = m_C + m_H*4;
  final constant Units.MolecularMass m_C2H6 = m_C*2 + m_H*6;
  final constant Units.MolecularMass m_C3H8 = m_C*3 + m_H*8;
  final constant Units.MolecularMass m_C4H10 = m_C*4 + m_H*10;
  final constant Units.MolecularMass m_CO2 = m_C + m_O*2;
  final constant Units.MolecularMass m_H2O = m_H*2 + m_O;

  // Heating values: based on ISO 6976 at 25°C
  // Methane CH4
  final constant Real hhv_molar_CH4 = 891.51 "kJ/mol";
  final constant Real hhv_mass_CH4 = hhv_molar_CH4/m_CH4 "MJ/kg";
  // Ethane C2H6
  final constant Real hhv_molar_C2H6 = 1562.06 "kJ/mol";
  final constant Real hhv_mass_C2H6 = hhv_molar_C2H6/m_C2H6 "MJ/kg";
  // Propane C3H8
  final constant Real hhv_molar_C3H8 = 2220.99 "kJ/mol";
  final constant Real hhv_mass_C3H8 = hhv_molar_C3H8/m_C3H8 "MJ/kg";
  // n-Butane C4H10
  final constant Real hhv_molar_C4H10 = 2879.63 "kJ/mol";
  final constant Real hhv_mass_C4H10 = hhv_molar_C4H10/m_C4H10 "MJ/kg";
  // Hydrogen H2
  final constant Real hhv_molar_H2 = 286.13 "kJ/mol";
  final constant Real hhv_mass_H2 = hhv_molar_H2/m_H2 "MJ/kg";


  // Heating values: based on ISO 6976 at 25°C
  // Methane CH4
  final constant Real lhv_molar_CH4 = 802.69 "kJ/mol";
  final constant Real lhv_mass_CH4 = lhv_molar_CH4/m_CH4 "MJ/kg";
  // Ethane C2H6
  final constant Real lhv_molar_C2H6 = 1428.83 "kJ/mol";
  final constant Real lhv_mass_C2H6 = lhv_molar_C2H6/m_C2H6 "MJ/kg";
  // Propane C3H8
  final constant Real lhv_molar_C3H8 = 2043.35 "kJ/mol";
  final constant Real lhv_mass_C3H8 = lhv_molar_C3H8/m_C3H8 "MJ/kg";
  // n-Butane C4H10
  final constant Real lhv_molar_C4H10 = 2657.58 "kJ/mol";
  final constant Real lhv_mass_C4H10 = lhv_molar_C4H10/m_C4H10 "MJ/kg";
  // Hydrogen H2
  final constant Real lhv_molar_H2 = 241.72 "kJ/mol";
  final constant Real lhv_mass_H2 = lhv_molar_H2/m_H2 "MJ/kg";

  annotation (
  Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
      Polygon(
        origin={-9.2597,25.6673},
        fillColor={102,102,102},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{48.017,11.336},{48.017,11.336},{10.766,11.336},{-25.684,10.95},{-34.944,-15.111},{-34.944,-15.111},{-32.298,-15.244},{-32.298,-15.244},{-22.112,0.168},{11.292,0.234},{48.267,-0.097},{48.267,-0.097}},
        smooth=Smooth.Bezier),
      Polygon(
        origin={-19.9923,-8.3993},
        fillColor={102,102,102},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{3.239,37.343},{3.305,37.343},{-0.399,2.683},{-16.936,-20.071},{-7.808,-28.604},{6.811,-22.519},{9.986,37.145},{9.986,37.145}},
        smooth=Smooth.Bezier),
      Polygon(
        origin={23.753,-11.5422},
        fillColor={102,102,102},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{-10.873,41.478},{-10.873,41.478},{-14.048,-4.162},{-9.352,-24.8},{7.912,-24.469},{16.247,0.27},{16.247,0.27},{13.336,0.071},{13.336,0.071},{7.515,-9.983},{-3.134,-7.271},{-2.671,41.214},{-2.671,41.214}},
        smooth=Smooth.Bezier)}));
end Constants;
