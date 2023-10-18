within MetroscopeModelingLibrary.Partial.Sensors;
partial model FlowSensor
  extends BaseSensor(faulty_flow_rate=faulty)                                   annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;

  parameter Units.VolumeFlowRate Qv_0 = Q_0/1000;

  Units.VolumeFlowRate Qv(start=Qv_0, nominal=Qv_0);
  Real Q_lm(start=Qv_0*Constants.m3s_to_lm, nominal=Qv_0*Constants.m3s_to_lm); // Flow rate in liter per minute;

  Real Q_th(start=Q_0*Constants.kgs_to_th, nominal=Q_0*Constants.kgs_to_th); // Flow rate in tons per hour
  Real Q_lbs(start=Q_0*Constants.kgs_to_lbs, nominal=Q_0*Constants.kgs_to_lbs); // Flow rate in pounds per second;
  Real Q_Mlbh(start=Q_0*Constants.kgs_to_Mlbh, nominal=Q_0*Constants.kgs_to_Mlbh); // Flow rate in pounds per second;

  // Failure modes
  parameter Boolean faulty = false;
equation
  Qv = Q / Medium.density(state);
  Q_lm = Qv * Constants.m3s_to_lm;
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
  Q_Mlbh = Q * Constants.kgs_to_Mlbh;
end FlowSensor;
