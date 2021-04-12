within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model PipePressureLoss "Pipe generic pressure loss"
   replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.PipePressureLoss(redeclare
      package Medium =
        WaterSteamMedium);
end PipePressureLoss;
