within MetroscopeModelingLibrary.FlueGases.BaseClasses;
model FlueGasesIsoHFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end FlueGasesIsoHFlowModel;
