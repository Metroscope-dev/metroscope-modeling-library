within MetroscopeModelingLibrary.Sensors.Displayer;
model WaterDisplayer
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  extends Partial.Sensors.Displayer(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

  annotation (Icon(graphics={Line(
          points={{-40,0},{42,0}},
          color={28,108,200},
          thickness=1)}));
end WaterDisplayer;
