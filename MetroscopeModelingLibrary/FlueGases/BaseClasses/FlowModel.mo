within MetroscopeModelingLibrary.FlueGases.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FlueGasesBaseClassIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium,
    Q_0 = 500, rho_0 = 1, medium=Types.Medium.FlueGases, plant=Types.Plant.CCGT) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Types;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;

  assert(medium == Types.Medium.FlueGases, "only flue gases medium config accepted for FlueGases flow model");
end FlowModel;
