within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model MoistAirFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BaseClasses.FlowModel(redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
                                        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
                                        redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end MoistAirFlowModel;
