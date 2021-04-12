within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model SingularPressureLoss
   replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss(redeclare
      package Medium =
        WaterSteamMedium);
end SingularPressureLoss;
