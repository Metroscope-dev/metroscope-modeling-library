within MetroscopeModelingLibrary.Fuel.PressureLosses;
model SingularPressureLoss
       package FuelMedium =
          MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
      extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss(          redeclare
      package Medium =
            FuelMedium);
end SingularPressureLoss;
