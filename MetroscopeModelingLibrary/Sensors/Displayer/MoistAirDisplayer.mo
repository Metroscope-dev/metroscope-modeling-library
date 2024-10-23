within MetroscopeModelingLibrary.Sensors.Displayer;
model MoistAirDisplayer
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

  extends Partial.Sensors.Displayer(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

  annotation (Icon(graphics={Line(
          points={{-40,0},{42,0}},
          color={85,170,255},
          thickness=1)}));
end MoistAirDisplayer;
