within MetroscopeModelingLibrary.FlueGases.PressureLosses;
model SingularPressureLoss
   replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss(redeclare
      package Medium =
        FlueGasesMedium);
end SingularPressureLoss;
