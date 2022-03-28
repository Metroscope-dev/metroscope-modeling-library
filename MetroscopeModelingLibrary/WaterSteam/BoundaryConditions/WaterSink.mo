within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSink
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSink(
                                          redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                          redeclare package Medium = WaterSteamMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end WaterSink;
