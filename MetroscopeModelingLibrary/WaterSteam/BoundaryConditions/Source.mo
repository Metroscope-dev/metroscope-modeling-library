within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model Source
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  extends MetroscopeModelingLibrary.Common.BoundaryConditions.Source(redeclare
      package                                                                        Medium =
        WaterSteamMedium);
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={63,81,181},
          fillPattern=FillPattern.Solid)}));
end Source;
