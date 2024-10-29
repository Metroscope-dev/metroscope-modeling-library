within MetroscopeModelingLibrary.DynamicComponents.Correlations;
model Test_2phase_function

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  // Input arguments
  input Units.Pressure p(start=127.34658e5) "Saturation ressure";
  input Units.PositiveMassFlowRate Q(start=150) "Mass flow rate";
  input Units.Power W(start=26725826) "Heat input";
  input Units.SpecificEnthalpy h(start=2021168.9) "Steam quality";
  input Units.Area A_c(start=1.2471651) "Tubes cross-sectional area";
  input Units.Area A(start=1049.4437) "Tubes exposed area";
  input Units.Length D_in(start=0.0328) "Tubes inner diameter";
  input Integer N_tubes(start=1476) "Number of tubes";

  output Units.HeatExchangeCoefficient K_conv;

  // Intermediate variables
  Real G;
  Real Bo "Boiling number";
  Real X_tt "Martinelli number";
  Real E "Enhancement factor";
  Real S "Suppression factor";
  Real Re_l "Liquid fraction Reynold's number";
  Real Re_s "Steam fraction Reynold's number";
  Real Pr_l "Liquid fraction Prandtl number";
  Real Pr_s "Steam fraction Prandtl number";
  Units.HeatExchangeCoefficient K_l "Liquid heat transfer coefficient";
  Units.HeatExchangeCoefficient K_s "Steam heat transfer coefficient";
  Units.HeatExchangeCoefficient K_pool "Pool boiling heat transfer coefficient";
  Real x "Steam quality";
  Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
  Units.Density rho_s "Steam density";
  Units.Density rho_l "Liquid density";
  Units.SpecificEnthalpy h_l "Liquid enthalpy";
  Units.SpecificEnthalpy h_s "Steam enthalpy";
  Modelica.Units.SI.DynamicViscosity Mu_l "Sat. liquid dynamic viscosity";
  Modelica.Units.SI.DynamicViscosity Mu_s "Sat. vapor dynamic viscosity";
  Units.HeatCapacity Cp_l "Sat. liquid heat capacity";
  Units.HeatCapacity Cp_s "Sat. vapor heat capacity";
  Modelica.Units.SI.ThermalConductivity k_l "Sat. liquid thermal conductivity";
  Modelica.Units.SI.ThermalConductivity k_s "Sat. vapor thermal conductivity";
  parameter Real Mmol = 18.015 "Water molar mass";
  parameter Units.Pressure pcrit = 220.64e5 "Critical pressure";
  parameter Real pi = Constants.pi;
  parameter Real x_min = 0.0002;
  parameter Real x_max = 0.85;

equation

  // Set saturation state
  sat.psat = p;
  sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

  // Saturation properties
  rho_s =WaterSteamMedium.density(WaterSteamMedium.setDewState(sat));
  rho_l =WaterSteamMedium.density(WaterSteamMedium.setBubbleState(sat));
  h_l =WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setBubbleState(sat));
  h_s =WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setDewState(sat));
  Mu_l =WaterSteamMedium.dynamicViscosity(WaterSteamMedium.setBubbleState(sat));
  Mu_s =WaterSteamMedium.dynamicViscosity(WaterSteamMedium.setDewState(sat));
  Cp_l =WaterSteamMedium.specificHeatCapacityCp(WaterSteamMedium.setBubbleState(sat));
  Cp_s =WaterSteamMedium.specificHeatCapacityCp(WaterSteamMedium.setDewState(sat));
  k_l =WaterSteamMedium.thermalConductivity(WaterSteamMedium.setBubbleState(sat));
  k_s =WaterSteamMedium.thermalConductivity(WaterSteamMedium.setDewState(sat));

  // Steam quality
  x = (h - h_l)/(h_s - h_l);
  // Reynold's number
  Re_l = abs(4*Q*(1 - x)/(pi*D_in*N_tubes*Mu_l));
  Re_s = abs(4*Q*x/(pi*D_in*N_tubes*Mu_s));
  // Prandtl number
  Pr_l = Cp_l*Mu_l/k_l;
  Pr_s = Cp_s*Mu_s/k_s;
  // Liquid fraction heat transfer coefficient
  K_l = 0.023*(abs(Re_l))^0.8*(abs(Pr_l))^0.4*k_l/D_in;
  K_s = 0.023*(abs(Re_s))^0.8*(abs(Pr_s))^0.4*k_s/D_in;
  // Mass flux
  G = abs(Q/A_c);
  // Boiling number
  Bo = abs(W/(A*(h_s - h_l)*G));
  // Enhancement factor
  E = 1 + 24000*Bo^1.16 + 1.37*(1/X_tt)^0.86;
  // Suppression factor
  S = 1/(1 + 1.15e-6*E^2*Re_l^1.17);
  // Pool boiling heat transfer coefficient
  K_pool = 55*(abs(p/pcrit))^0.12*(-Modelica.Math.log10(abs(p/pcrit)))^(-0.55)*Mmol^(-0.5)*(abs(W/A))^0.67;
  // Martellini number and convection HTC
  if (x < x_min) then
    X_tt = (abs((1 - x_min)/x_min))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
    if (p > pcrit) then
      K_conv = K_l;
    else
      K_conv = (1 - x)/x_min*K_l + x/x_min*(E*K_l + S*K_pool);
    end if;

  elseif (x > x_max) then
    X_tt = (abs((1 - x_max)/x_max))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
    K_conv = (x - x_max)/(1 - x_max)*K_s + (1 - x)/(1 - x_max)*(E*K_l + S*K_pool);

  else
    X_tt = (abs((1 - x)/x))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
    K_conv = E*K_l + S*K_pool;

  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Test_2phase_function;
