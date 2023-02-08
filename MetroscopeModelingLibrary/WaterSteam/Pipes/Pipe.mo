within MetroscopeModelingLibrary.WaterSteam.Pipes;
model Pipe
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Pipes.Pipe(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                             Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pipe;
