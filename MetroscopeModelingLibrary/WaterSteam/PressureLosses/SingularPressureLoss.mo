within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model SingularPressureLoss
   package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss(redeclare
      package Medium =
        WaterSteamMedium);
end SingularPressureLoss;
