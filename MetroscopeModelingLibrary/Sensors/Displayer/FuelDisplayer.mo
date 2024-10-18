within MetroscopeModelingLibrary.Sensors.Displayer;
model FuelDisplayer
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

  extends Partial.Sensors.Displayer(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

  annotation (Icon(graphics={Line(
          points={{-40,0},{42,0}},
          color={213,213,0},
          thickness=1)}));
end FuelDisplayer;
