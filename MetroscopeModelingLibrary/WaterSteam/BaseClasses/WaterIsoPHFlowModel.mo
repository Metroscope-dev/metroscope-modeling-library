within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoPHFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.WaterSteamBaseClassIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(primitivesVisible=false);
end WaterIsoPHFlowModel;
