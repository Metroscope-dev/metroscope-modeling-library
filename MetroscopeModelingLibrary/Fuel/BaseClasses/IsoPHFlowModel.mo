within MetroscopeModelingLibrary.Fuel.BaseClasses;
model IsoPHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FuelBaseClassIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end IsoPHFlowModel;
