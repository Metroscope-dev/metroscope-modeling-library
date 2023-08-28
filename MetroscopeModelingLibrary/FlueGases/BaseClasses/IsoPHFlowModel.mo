within MetroscopeModelingLibrary.FlueGases.BaseClasses;
model IsoPHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FlueGasesBaseClassIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium,
    Q_0 = 500, rho_0=1) annotation (IconMap(primitivesVisible=false));
end IsoPHFlowModel;
