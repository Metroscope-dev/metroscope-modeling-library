within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Sink
  package MoistAirMedium =
          MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
      extends MetroscopeModelingLibrary.Common.BoundaryConditions.Sink(redeclare
      package     Medium =
            MoistAirMedium, h_vol(start=3e5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Sink;
