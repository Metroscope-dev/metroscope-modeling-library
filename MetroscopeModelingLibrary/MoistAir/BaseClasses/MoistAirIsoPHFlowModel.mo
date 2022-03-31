within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model MoistAirIsoPHFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(h_0=1e3, P_0=0.9e5,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
    redeclare package Medium = MoistAirMedium, medium_name="MoistAir") annotation(IconMap(primitivesVisible=false));
end MoistAirIsoPHFlowModel;
