within MetroscopeModelingLibrary.WaterSteam.PressureLosses;
model SingularPressureAndHeatLoss
   package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends
    MetroscopeModelingLibrary.Common.Partial.FlowModel(      redeclare package
              Medium =
        WaterSteamMedium);
end SingularPressureAndHeatLoss;
