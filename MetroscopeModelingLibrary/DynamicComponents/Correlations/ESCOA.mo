within MetroscopeModelingLibrary.DynamicComponents.Correlations;
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
