within MetroscopeModelingLibrary.Partial.Sensors_Control;
partial model FlowSensor
  extends BaseSensor(faulty_flow_rate=faulty)                                   annotation(IconMap(primitivesVisible=true));
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;

  parameter Units.VolumeFlowRate Qv_0 = Q_0/1000;

  parameter Real init_Q = 1;

  Units.VolumeFlowRate Qv(start=Qv_0, nominal=Qv_0);
  Real Q_lm(start=Qv_0*Constants.m3s_to_lm, nominal=Qv_0*Constants.m3s_to_lm); // Flow rate in liter per minute;

  Real Q_th(start=Q_0*Constants.kgs_to_th, nominal=Q_0*Constants.kgs_to_th); // Flow rate in tons per hour
  Real Q_lbs(start=Q_0*Constants.kgs_to_lbs, nominal=Q_0*Constants.kgs_to_lbs); // Flow rate in pounds per second;
  Real Q_Mlbh(start=Q_0*Constants.kgs_to_Mlbh, nominal=Q_0*Constants.kgs_to_Mlbh); // Flow rate in pounds per second;

  // Failure modes
  parameter Boolean faulty = false;

  parameter String display_unit = "kg/s" "Specify the display unit"
    annotation(choices(choice="kg/s", choice="m3/s", choice="l/m", choice="t/h", choice="lb/s", choice="Mlb/h"));
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
  parameter String signal_unit = "kg/s" annotation (choices(choice="kg/s", choice="l/m", choice="t/h"));

  Utilities.Interfaces.GenericReal      Q_sensor(start=init_Q) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,100}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,100})));
equation
  Qv = Q / Medium.density(state);
  Q_lm = Qv * Constants.m3s_to_lm;
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
  Q_Mlbh = Q * Constants.kgs_to_Mlbh;

  if signal_unit == "l/m" then
    Q_sensor = Q_lm;
  elseif signal_unit == "t/h" then
    Q_sensor = Q_th;
  else
    Q_sensor = Q;
  end if;

  connect(Q_sensor, Q_sensor) annotation (Line(points={{0,100},{0,100}}, color={0,0,127}));
  annotation (Icon(graphics={Text(
        extent={{-100,-160},{102,-200}},
        textColor={0,0,0},
        textString=if display_output then
                   if display_unit == "m3/s" then DynamicSelect("",String(Qv)+" m3/s")
                   else if display_unit == "l/m" then DynamicSelect("",String(Q_lm)+" l/m")
                   else if display_unit == "t/h" then DynamicSelect("",String(Q_th)+" t/h")
                   else if display_unit == "lb/s" then DynamicSelect("",String(Q_lbs)+" lb/s")
                   else if display_unit == "Mlb/h" then DynamicSelect("",String(Q_Mlbh)+" Mlb/h")
                   else DynamicSelect("",String(Q)+" kg/s")
                   else "")}));
end FlowSensor;
