within MetroscopeModelingLibrary.DynamicComponents.Correlations;
function Kandlikar_2phase
  "Computes the 2 phase heat transfer Coefficient for vertical tubes (c_5 = 0) with water inside (F_k = 1) based on Kandlikar empirical relation"

  import MetrosCopeModelingLibrary.Utilities.Units;
  import MetrosCopeModelingLibrary.Utilities.Constants;
  package WaterSteamMedium = MetrosCopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  // Input arguments
  input Units.Pressure p "Saturation ressure";
  input Units.PositiveMassFlowRate Q "Mass flow rate";
  input Units.Power W "Heat input";
  input Units.HeatExchangeCoefficient k_1phase "1 phase heat transfer Coefficient";
  input Real x "Steam quality";
  input Units.Area A_c "Tubes cross-sectional area";
  input Units.Area A "Tubes exposed area";

  output Units.HeatExchangeCoefficient k_2phase;

  // Intermediate variables
protected
  Real G;
  Real Co "Convection number";
  Real Bo "Boiling number";
  Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
  Units.Density rho_s "Steam density";
  Units.Density rho_l "Liquid density";
  Units.SpecificEnthalpy h_l "Liquid enthalpy";
  Units.SpecificEnthalpy h_s "Steam enthalpy";
  Real c_1, c_2, c_3, c_4;
  //parameter Real g = Constants.g;

algorithm

  // Set saturation state
  sat.psat := p;
  sat.Tsat := Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

  // Saturation properties
  rho_s :=WaterSteamMedium.density(WaterSteamMedium.setDewState(sat));
  rho_l :=WaterSteamMedium.density(WaterSteamMedium.setBubbleState(sat));
  h_l :=WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setBubbleState(sat));
  h_s :=WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setDewState(sat));

  // Constants
  c_4 :=0.7;
  if (Co < 0.65) then
    c_1 :=1.136;
    c_2 :=-0.9;
    c_3 :=667.2;
  else
    c_1 :=0.6683;
    c_2 :=-0.2;
    c_3 :=1058;
  end if;

  G := Q/A_c;
  Bo := W/(A*G*(h_s - h_l));

  if x > 0 then
    Co := ((1 - x)/x)^0.8*(rho_s/rho_l)^0.5;
    k_2phase := k_1phase*(c_1*Co^c_2 + c_3*abs(Bo)^c_4);
  else
    k_2phase := k_1phase;
    Co :=1;
  end if;

end Kandlikar_2phase;
