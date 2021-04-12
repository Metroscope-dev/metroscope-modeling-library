within MetroscopeModelingLibrary.MoistAir.BoundaryConditions;
model Source

  replaceable package MoistAirMedium =
          MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
      extends MetroscopeModelingLibrary.Common.BoundaryConditions.Source(redeclare
      package     Medium =
            MoistAirMedium, h_vol(start=283945));

  Real relative_humidity(start=0.1);


equation


 Xi_vol[1] = MoistAirMedium.massFraction_pTphi(P_out, T_vol, relative_humidity);



  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Source;
