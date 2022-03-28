within MetroscopeModelingLibrary.Partial.Sensors;
partial model FlowSensor
  extends Partial.BaseClasses.IsoPHFlowModel annotation(IconMap(primitivesVisible=false));
  extends FluidSensorIcon;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;

  Real Q_th(unit="1/h", start=Q_0*Constants.kgs_to_th, nominal=Q_0*Constants.kgs_to_th); // Flow rate in tons per hour
  Real Q_lbs(unit="1/h", start=Q_0*Constants.kgs_to_lbs, nominal=Q_0*Constants.kgs_to_lbs); // Flow rate in pounds per second;
equation
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                             Text(
          extent={{-40,52},{42,-56}},
          textColor={0,0,0},
          textString="Q")}),                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
end FlowSensor;
