within MetroscopeModelingLibrary.MoistAir.PressureLosses;
model SingularPressureLoss
  replaceable package MoistAirMedium =
          MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
      extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss(          redeclare
      package Medium =
            MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SingularPressureLoss;
