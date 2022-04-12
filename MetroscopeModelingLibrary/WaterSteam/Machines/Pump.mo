within MetroscopeModelingLibrary.WaterSteam.Machines;
model Pump
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Machines.Pump(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium);
end Pump;
