within MetroscopeModelingLibrary.DynamicComponents;
package Correlations

  function Zukauskas "External flow heat transfer coefficient - Bare tubes"

    import MetroscopeModelingLibrary.Utilities.Units;

    // Input arguments
      input Real Re_fg_max;
      input Real Pr_fg_avg;
      input Real Pr_fg_avg_s;
      input Integer Tubes_Config;
      input Integer Rows;
      input Units.Length S_T;
      input Units.Length S_L;

    // Function output
      // Nusselt number
      output Real Nu_fg_avg;

  protected
    Real Constant_C "Constant used in the Zukauskas empirical relation";
    Real Constant_m "Constant used in the Zukauskas empirical relation";
    parameter Real C2[2,19] = {{0.7,0.8,0.86,0.9,0.92,0.935,0.95,0.956666667,0.963333333,0.97,0.973333333,0.976666667,0.98,0.983333333,0.986666667,0.99,0.99,0.99,0.99},
                               {0.64,0.76,0.84,0.89,0.92,0.935,0.95,0.956666667,0.963333333,0.97,0.973333333,0.976666667,0.98,0.983333333,0.986666667,0.99,0.99,0.99,0.99}};

  algorithm

    if (Tubes_Config == 1) then
      if (Re_fg_max > 10 and Re_fg_max < 10^2) then
        Constant_C := 0.8;
        Constant_m := 0.4;
      elseif (Re_fg_max > 10^3 and Re_fg_max < 2*10^5) then
        Constant_C := 0.27;
        Constant_m := 0.63;
      elseif (Re_fg_max > 2*10^5 and Re_fg_max < 2*10^6) then
        Constant_C := 0.021;
        Constant_m := 0.84;
      end if;

    elseif  (Tubes_Config == 2) then
      if (Re_fg_max > 10 and Re_fg_max < 10^2) then
        Constant_C := 0.9;
        Constant_m := 0.4;
      elseif (Re_fg_max > 10^3 and Re_fg_max < 2*10^5) then
        if (S_T/S_L < 2) then
          Constant_C := 0.35*(S_T/S_L)^(1/5);
          Constant_m := 0.6;
        else
          Constant_C := 0.4;
          Constant_m := 0.6;
        end if;
      elseif (Re_fg_max > 2*10^5 and Re_fg_max < 2*10^6) then
        Constant_C := 0.022;
        Constant_m := 0.84;
      end if;
    end if;

    if (Rows < 20) then
      Nu_fg_avg :=Constant_C*Re_fg_max^Constant_m*Pr_fg_avg^0.36*(Pr_fg_avg/Pr_fg_avg_s)^0.25*C2[Tubes_Config, Rows];
    else
      Nu_fg_avg :=Constant_C*Re_fg_max^Constant_m*Pr_fg_avg^0.36*(Pr_fg_avg/Pr_fg_avg_s)^0.25;
    end if;

  end Zukauskas;

  function ESCOA "External flow heat transfer coefficient - Finned tubes"

    /* This function is valid for segmented finned-tubes at staggered arrangement with:
    2000 < Re_fg < 500000
    9.5 mm < H_fin < 38.1 mm
    0.9 mm < e_fin < 4.2 mm
    39.37 fin/m < S_fin < 275 fin/m
  */

    import MetroscopeModelingLibrary.Utilities.Units;

    // Input arguments
      input Real Re_fg "Flue gas Reynold's number";
      input Real Pr_fg "Flue gas Prandtl number";
      input Integer Rows "Number of tubes in flow direction";
      input Units.Temperature T_fg "Flue gas temperature";
      input Units.Temperature T_fin "Fin temperature";
      input Units.Length D_out "Tube outer diameter";
      input Units.Length H_fin "Fin height";
      input Units.Length e_fin "Fin thickness";
      input Units.Length S_fin "Fin pitch";
      input Units.Length S_T "Tube bundle transverse pitch";
      input Units.Length S_L "Tube bundle longitudanal pitch";

    // Function output
      // Nusselt number
      output Real Nu_fg;

  protected
    Real Constant_C1 "Constant used in ESCOA empirical relation";
    Real Constant_C3 "Constant used in ESCOA empirical relation";
    Real Constant_C5 "Constant used in ESCOA empirical relation";
    parameter Units.Length D_fin = D_out + 2*H_fin "Fin outer diameter";

  algorithm

     // Traditional ESCOA Correlation
    Constant_C1 := 0.25*Re_fg^(-0.35);
    Constant_C3 := 0.55 + 0.45*exp(-0.35*H_fin/(S_fin - e_fin));
    Constant_C5 := 0.7 + (0.7 - 0.8*exp(-0.15*Rows^2))*exp(-S_L/S_T);

    // Revised ESCOA Correlation

  //   Constant_C1 := 0.091*Re_fg^(-0.25);
  //   Constant_C3 := 0.35 + 0.65*exp(-0.17*H_fin/(S_fin - e_fin));
  //   Constant_C5 := 0.7 + (0.7 - 0.8*exp(-0.15*Rows^2))*exp(-S_L/S_T);

    Nu_fg := Constant_C1*Constant_C3*Constant_C5*Re_fg*Pr_fg^(1/3)*((T_fg + 273.15)/(T_fin + 273.15))^0.25*(D_fin/D_out)^0.5;

  end ESCOA;

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

  function Gungor_Winterton_2phase

    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Constants;
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    // Input arguments
    input Units.Pressure p "Saturation ressure";
    input Units.PositiveMassFlowRate Q "Mass flow rate";
    input Units.Power W "Heat input";
    input Real x "Steam quality";
    input Units.Area A_c "Tubes cross-sectional area";
    input Units.Area A "Tubes exposed area";
    input Units.Length D_in "Tubes inner diameter";
    input Units.Length L "Tubes length";
    input Integer N "number of nodes";
    input Integer N_tubes "Number of tubes";

    output Units.HeatExchangeCoefficient K_conv;

  protected
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

  algorithm

    // Set saturation state
    sat.psat :=p;
    sat.Tsat :=Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

    // Saturation properties
    rho_s :=WaterSteamMedium.density(WaterSteamMedium.setDewState(sat));
    rho_l :=WaterSteamMedium.density(WaterSteamMedium.setBubbleState(sat));
    h_l :=WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setBubbleState(sat));
    h_s :=WaterSteamMedium.specificEnthalpy(WaterSteamMedium.setDewState(sat));
    Mu_l :=WaterSteamMedium.dynamicViscosity(WaterSteamMedium.setBubbleState(sat));
    Mu_s :=WaterSteamMedium.dynamicViscosity(WaterSteamMedium.setDewState(sat));
    Cp_l :=WaterSteamMedium.specificHeatCapacityCp(WaterSteamMedium.setBubbleState(sat));
    Cp_s :=WaterSteamMedium.specificHeatCapacityCp(WaterSteamMedium.setDewState(sat));
    k_l :=WaterSteamMedium.thermalConductivity(WaterSteamMedium.setBubbleState(sat));
    k_s :=WaterSteamMedium.thermalConductivity(WaterSteamMedium.setDewState(sat));

    // Reynold's number
    Re_l :=abs(4*Q*(1 - x)/(pi*D_in*N_tubes*Mu_l));
    Re_s :=abs(4*Q*x/(pi*D_in*N_tubes*Mu_s));
    // Prandtl number
    Pr_l :=Cp_l*Mu_l/k_l;
    Pr_s :=Cp_s*Mu_s/k_s;
    // Liquid fraction heat transfer coefficient
    K_l :=0.023*(abs(Re_l))^0.8*(abs(Pr_l))^0.4*k_l/D_in;
    K_s :=0.023*(abs(Re_s))^0.8*(abs(Pr_s))^0.4*k_s/D_in;
    // Mass flux
    G :=abs(Q/A_c);
    // Boiling number
    Bo :=abs(W/(A*(h_s - h_l)*G));
    //Bo := abs(W*D_in/(4*Q*(h_s - h_l)*L/N));
    // Enhancement factor
    //E = 1 + 24000*Bo^1.16 + 1.37*(1/X_tt)^0.86;
    // Suppression factor
    //S :=1/(1 + 1.15e-6*E^2*Re_l^1.17);
    // Pool boiling heat transfer coefficient
    K_pool :=55*(abs(p/pcrit))^0.12*(-Modelica.Math.log10(abs(p/pcrit)))^(-0.55)*Mmol^(-0.5)*(abs(W/A))^0.67;
    // Martellini number and convection HTC
    if (x < x_min) then
      X_tt :=(abs((1 - x_min)/x_min))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
      E :=1 + 24000*Bo^1.16 + 1.37*(1/X_tt)^0.86;
      S :=1/(1 + 1.15e-6*E^2*Re_l^1.17);
      if (p > pcrit) then
        K_conv :=K_l;
      else
        K_conv :=(1 - x)/x_min*K_l + x/x_min*(E*K_l + S*K_pool);
      end if;

    elseif (x > x_max) then
      X_tt :=(abs((1 - x_max)/x_max))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
      E :=1 + 24000*Bo^1.16 + 1.37*(1/X_tt)^0.86;
      S :=1/(1 + 1.15e-6*E^2*Re_l^1.17);
      K_conv :=(x - x_max)/(1 - x_max)*K_s + (1 - x)/(1 - x_max)*(E*K_l + S*K_pool);

    else
      X_tt :=(abs((1 - x)/x))^0.9*(abs(rho_s/rho_l))^0.5*(abs(Mu_l/Mu_s))^0.1;
      E := 1 + 24000*Bo^1.16 + 1.37*(1/X_tt)^0.86;
      S := 1/(1 + 1.15e-6*E^2*Re_l^1.17);
      K_conv := E*K_l + S*K_pool;

    end if;

  end Gungor_Winterton_2phase;

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

  model Test_2phase_function1

    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Constants;
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
    import CorrelationConstants = MetroscopeModelingLibrary.DynamicComponents.Correlations;

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

  equation

    K_conv = CorrelationConstants.Gungor_Winterton_2phase(p, Q, W, h, A_c, A, D_in, N_tubes);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Test_2phase_function1;

  function removeSensorSuffix
    input String name;
    output String newName;

  algorithm
      newName := if Modelica.Utilities.Strings.find(name, "_sensor") > 0 then
      Modelica.Utilities.Strings.replace(name, "_sensor", "")
    else
      name;

  end removeSensorSuffix;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Text(
          extent={{100,100},{-100,-100}},
          textColor={0,0,0},
          fontName="Centaur",
          textString="f")}));
end Correlations;
