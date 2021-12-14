within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model PressureCut
   package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.PressureCut(redeclare
      package Medium =
        WaterSteamMedium);
end PressureCut;
