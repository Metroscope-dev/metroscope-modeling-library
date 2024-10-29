within MetroscopeModelingLibrary.DynamicComponents.Correlations;
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
