within MetroscopeModelingLibrary.DynamicComponents.Correlations;
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
