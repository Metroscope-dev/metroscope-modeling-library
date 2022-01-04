within MetroscopeModelingLibrary.MoistAir.Sensors;
model MoistAirHumiditySensor
    package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;

    extends MetroscopeModelingLibrary.Common.Sensors.BaseSensor(redeclare
      package
      Medium =
        MoistAirMedium);


    Real humidity;
    Real humidity_pc;

equation

    Xi_in[1] = MoistAirMedium.massFraction_pTphi(P_in, T_in, humidity);
    humidity_pc = 100*humidity;


  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                             Text(
          extent={{-106,46},{110,-48}},
          textColor={0,0,0},
          textString="X")}),                                     Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MoistAirHumiditySensor;
