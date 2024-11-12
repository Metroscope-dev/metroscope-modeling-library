within MetroscopeModelingLibrary.Partial.Sensors;
partial model FlowSensor
  extends BaseSensor(faulty_flow_rate=faulty)                                   annotation(IconMap(primitivesVisible=true));
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
  Real Q_lbh(start=Q_0*Constants.kgs_to_lbh, nominal= Q_0*Constants.kgs_to_lbh); // Flow rate in pounds per hour
  Real Q_ft3h(start=Qv_0*Constants.m3s_to_ft3h, nominal= Q_0*Constants.m3s_to_ft3h); // Volumetric flow rate in cubic foot per hour

  // Failure modes
  parameter Boolean faulty = false;

  parameter String display_unit = "kg/s" "Specify the display unit"
    annotation(choices(choice="kg/s", choice="m3/s", choice="l/m", choice="t/h", choice="lb/s", choice="Mlb/h"));
  parameter Boolean display_output = true "Used to switch ON or OFF output display";

equation
  Qv = Q / Medium.density(state);
  Q_lm = Qv * Constants.m3s_to_lm;
  Q_th = Q * Constants.kgs_to_th;
  Q_lbs = Q * Constants.kgs_to_lbs;
  Q_Mlbh = Q * Constants.kgs_to_Mlbh;
  Q_lbh = Q * Constants.kgs_to_lbh;
  Q_ft3h = Qv * Constants.m3s_to_ft3h;

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
