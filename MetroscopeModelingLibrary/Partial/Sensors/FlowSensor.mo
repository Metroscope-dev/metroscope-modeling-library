within MetroscopeModelingLibrary.Partial.Sensors;
partial model FlowSensor
  extends Partial.BaseClasses.IsoPHFlowModel annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Icons.Sensors.FluidSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlowIcon;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;

  Real Q_th(start=Q_0*Constants.kgs_to_th, nominal=Q_0*Constants.kgs_to_th); // Flow rate in tons per hour
  Real Q_lbs(start=Q_0*Constants.kgs_to_lbs, nominal=Q_0*Constants.kgs_to_lbs); // Flow rate in pounds per second;
equation
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
end FlowSensor;
