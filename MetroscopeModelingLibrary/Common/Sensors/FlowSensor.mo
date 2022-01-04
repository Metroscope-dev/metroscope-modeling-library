within MetroscopeModelingLibrary.Common.Sensors;
model FlowSensor
  extends BaseSensor;

  Real Q;  // Flow rate in kg/s (SI Unit)
  Real Q_th; // Flow rate in tons per hour
  Real Q_lbs; // Flow rate in pounds per second;

equation

  Q = Q_in;
  Q_th = (Q_in/1000)*3600;
  Q_lbs = Q_in*0.453592428;

  annotation (Icon(graphics={Text(
          extent={{-104,48},{112,-46}},
          textColor={0,0,0},
          textString="Q")}));
end FlowSensor;
