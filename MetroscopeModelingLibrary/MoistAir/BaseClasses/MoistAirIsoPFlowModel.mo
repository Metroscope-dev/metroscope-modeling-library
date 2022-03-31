within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model MoistAirIsoPFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
    redeclare package Medium = MoistAirMedium, medium_name="MoistAir") annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end MoistAirIsoPFlowModel;
