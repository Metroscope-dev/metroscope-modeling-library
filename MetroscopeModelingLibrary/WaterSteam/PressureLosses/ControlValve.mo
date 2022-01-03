within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model ControlValve
   package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.ControlValve(redeclare
      package Medium =
        WaterSteamMedium);
end ControlValve;
