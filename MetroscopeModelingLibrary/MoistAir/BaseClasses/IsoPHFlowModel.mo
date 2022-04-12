within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model IsoPHFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));
end IsoPHFlowModel;
