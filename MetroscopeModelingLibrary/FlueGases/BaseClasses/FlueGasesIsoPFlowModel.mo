within MetroscopeModelingLibrary.FlueGases.BaseClasses;
model FlueGasesIsoPFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end FlueGasesIsoPFlowModel;
