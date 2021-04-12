within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model LoopCloser
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.BoundaryConditions.LoopCloser(redeclare
      package Medium =
        WaterSteamMedium);
end LoopCloser;
