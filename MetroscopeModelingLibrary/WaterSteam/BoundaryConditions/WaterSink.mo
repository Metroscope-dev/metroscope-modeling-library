within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.WaterSinkIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSink(
                                          redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                          redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end WaterSink;
