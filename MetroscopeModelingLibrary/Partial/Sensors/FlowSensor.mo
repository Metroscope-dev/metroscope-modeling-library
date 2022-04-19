within MetroscopeModelingLibrary.Partial.Sensors;
partial model FlowSensor
  extends Partial.BaseClasses.IsoPHFlowSimplifiedModel annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Icons.Sensors.FluidSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlowIcon;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;

  parameter Units.VolumeFlowRate Qv_0 = Q_0/1000;

  Units.VolumeFlowRate Qv(start=Qv_0, nominal=Qv_0);
  Real Q_th(start=Q_0*Constants.kgs_to_th, nominal=Q_0*Constants.kgs_to_th); // Flow rate in tons per hour
  Real Q_lbs(start=Q_0*Constants.kgs_to_lbs, nominal=Q_0*Constants.kgs_to_lbs); // Flow rate in pounds per second;
equation
  Qv = Q / Medium.density(state);
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
end FlowSensor;
