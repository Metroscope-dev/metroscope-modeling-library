within MetroscopeModelingLibrary.Sensors.Displayer;
model FlueGasesDisplayer
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  extends Partial.Sensors.Displayer(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

  annotation (Icon(graphics={Line(
          points={{-40,0},{42,0}},
          color={95,95,95},
          thickness=1)}));
end FlueGasesDisplayer;
