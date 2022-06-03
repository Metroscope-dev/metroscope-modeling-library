within MetroscopeModelingLibrary.FlueGases.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.FlueGasesBaseClassIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end FlowModel;
