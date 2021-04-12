within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model SingularPressureAndHeatLoss
   replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureAndHeatLoss(      redeclare
      package Medium =
        WaterSteamMedium);
end SingularPressureAndHeatLoss;
