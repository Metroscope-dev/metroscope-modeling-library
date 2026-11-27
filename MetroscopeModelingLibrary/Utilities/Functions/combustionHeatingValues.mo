within MetroscopeModelingLibrary.Utilities.Functions;
function combustionHeatingValues "Higher heating value and water condensation correction per kg of fuel"
  extends Modelica.Icons.Function;
  import Modelica.Units.SI;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import Data = Modelica.Media.IdealGases.Common.SingleGasesData;

  input SI.MassFraction hydrocarbons[4] "Mass fractions of CH4, C2H6, C3H8 and n-C4H10";
  input SI.MassFraction hydrogen = 0 "Mass fraction of H2";
  input SI.MassFraction carbonMonoxide = 0 "Mass fraction of CO";
  output SI.SpecificEnthalpy HHV "Higher heating value at 298.15 K";
  output SI.SpecificEnthalpy condensationHeat "Heat of condensation of the water formed";

protected
  constant SI.SpecificEnthalpy condensationPerHydrogenMass = 2202.92069*1e4
    "Water condensation correction per kg of hydrogen in the fuel, as used in CombustionChamber";
  constant SI.MassFraction boundHydrogen[4] = Constants.m_H*
    ({4,6,8,10} ./ {Constants.m_CH4,Constants.m_C2H6,Constants.m_C3H8,Constants.m_C4H10});
  constant SI.SpecificEnthalpy hydrocarbonHHV[4] = 1e6*
    {Constants.hhv_mass_CH4,Constants.hhv_mass_C2H6,Constants.hhv_mass_C3H8,Constants.hhv_mass_C4H10};
  constant SI.SpecificEnthalpy hydrogenHHV =
    (Data.H2.Hf*Data.H2.MM + 0.5*Data.O2.Hf*Data.O2.MM - Data.H2O.Hf*Data.H2O.MM)/Data.H2.MM
    + condensationPerHydrogenMass
    "Hydrogen heat of reaction with the same water condensation convention";
  constant SI.SpecificEnthalpy carbonMonoxideHHV =
    (Data.CO.Hf*Data.CO.MM + 0.5*Data.O2.Hf*Data.O2.MM - Data.CO2.Hf*Data.CO2.MM)/Data.CO.MM;

algorithm
  HHV := hydrocarbons*hydrocarbonHHV + hydrogen*hydrogenHHV + carbonMonoxide*carbonMonoxideHHV;
  condensationHeat := (hydrocarbons*boundHydrogen + hydrogen)*condensationPerHydrogenMass;
end combustionHeatingValues;
