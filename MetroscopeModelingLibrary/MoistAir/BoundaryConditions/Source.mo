within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Source

  package MoistAirMedium =
          MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
      extends MetroscopeModelingLibrary.Common.BoundaryConditions.Source(redeclare
      package     Medium =
            MoistAirMedium);

  Real relative_humidity(start=0.1);


equation


 Xi_out[1] = MoistAirMedium.massFraction_pTphi(P_out, T_out, relative_humidity);



  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Source;
