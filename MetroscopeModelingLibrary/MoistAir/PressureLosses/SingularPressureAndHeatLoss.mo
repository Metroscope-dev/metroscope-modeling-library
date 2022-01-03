within MetroscopeModelingLibrary.MoistAir.PressureLosses;
model SingularPressureAndHeatLoss
   package MoistAirMedium =
          MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureAndHeatLoss(      redeclare
      package Medium =
        MoistAirMedium);
end SingularPressureAndHeatLoss;
