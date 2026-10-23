within MetroscopeModelingLibrary.Partial;
package SensorsControl

  partial model TemperatureSensor
    extends MetroscopeModelingLibrary.Partial.Sensors.TemperatureSensor                                   annotation(IconMap(primitivesVisible=true));
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Constants;

    // Sensor signal parameters
    parameter Real T_start = 300 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
    parameter String signal_unit = "degC" "Specify the signal unit. This should be the unit of T_start and of the tag linked to the sensor." annotation (choices(choice="degC", choice="K", choice="degF"),
    Dialog(tab="General", group="Sensor signal parameters"));
    parameter String display_unit = "degC" "Specify the display unit"
    annotation(choices(choice="degC", choice="K", choice="degF"), Dialog(tab="General", group="Sensor signal parameters"));

    parameter Boolean display_output = true "Used to switch ON or OFF output display";

    Utilities.Interfaces.GenericReal      T_sensor(start=T_start) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation


    if signal_unit == "degC" then
      T_sensor = T_degC;
    elseif signal_unit == "degF" then
      T_sensor = T_degF;
    elseif signal_unit == "K" then
      T_sensor = T;
    else
      assert(false, "Unavailable unit selected for %name");
    end if;

    annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     if display_unit == "K" then DynamicSelect("",String(T)+" K")
                     else if display_unit == "degF" then DynamicSelect("",String(T_degF)+" degF")
                     else DynamicSelect("",String(T_degC)+" degC")
                     else "")}));
  end TemperatureSensor;

  partial model PressureSensor
    extends Sensors.PressureSensor                         annotation(IconMap(primitivesVisible=true));

    // Sensor signal parameters
    parameter String signal_unit = "barA" "Specify the signal unit. This should be the unit of P_start and of the tag linked to the sensor." annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA",choice="psiA",choice="psiG",choice="inHg", choice="inH2O"),
    Dialog(tab="General", group="Sensor signal parameters"));
    Real P_barG(nominal=P_0, start=P_0*Constants.Pa_to_barA - Constants.atmospheric_pressure_in_bar);
    Real P_psiG(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA - Constants.P0_psiG_in_psiA);
    Real P_MPaG(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA - Constants.P0_MPaG_in_MPaA);
    Real P_kPaG(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA - Constants.P0_kPaG_in_kPaA);
    Real P_barA(nominal = P_0*Constants.Pa_to_barA, start = P_0*Constants.Pa_to_barA, unit="bar");
    Real P_psiA(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA);
    Real P_MPaA(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA);
    Real P_kPaA(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA);
    Real P_inHg(nominal = P_0*Constants.Pa_to_inHg, start = P_0*Constants.Pa_to_inHg);
    Real P_inH2O(nominal = P_0*Constants.Pa_to_inH2O, start = P_0*Constants.Pa_to_inH2O);
    Real P_mbar(nominal = P_0*Constants.Pa_to_mbar, start = P_0*Constants.Pa_to_mbar, unit="mbar");
     parameter Boolean display_output = true "Used to switch ON or OFF output display";
  Utilities.Interfaces.GenericReal     P_sensor(start=P_start)
    annotation (Placement(transformation(
      extent={{-10,-10},{10,10}},
      rotation=90,
      origin={0,100}), iconTransformation(
      extent={{-10,-10},{10,10}},
      rotation=90,
      origin={0,100})));
  equation
    if signal_unit == "barA" then
      P_sensor = P_barA;
    elseif signal_unit == "barG" then
      P_sensor = P_barG;
    elseif signal_unit == "mbar" then
      P_sensor = P_mbar;
    elseif signal_unit == "MPaA" then
      P_sensor = P_MPaA;
    elseif signal_unit == "kPaA" then
      P_sensor = P_kPaA;
    elseif signal_unit == "psiG" then
      P_sensor=P_psiG;
    elseif signal_unit == "psiA" then
      P_sensor=P_psiA;
    elseif signal_unit == "inHg" then
      P_sensor=P_inHg;
    elseif signal_unit == "inH2O" then
      P_sensor=P_inH2O;
    else
      P_sensor = P;
    end if;
      annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA",choice="psiA",choice="psiG",choice="inHg",choice="inH2O"),
      Dialog(tab="General", group="Sensor signal parameters"),
                Icon(graphics={Text(
            extent={{-100,-160},{102,-200}},
            textColor={0,0,0},
            textString=if display_output then
                       if display_unit == "barG" then DynamicSelect("",String(P_barG)+" barG")
                       else if display_unit == "mbar" then DynamicSelect("",String(P_mbar)+" mbar")
                       else if display_unit == "MPaA" then DynamicSelect("",String(P_MPaA)+" MPaA")
                       else if display_unit == "kPaA" then DynamicSelect("",String(P_kPaA)+" kPaA")
                       else DynamicSelect("",String(P_barA)+" barA")
                       else "")}));

    // All nominal values of gauge pressures are set to the absolute pressure value to avoid zero nominal value
                                                                                                      // Relative (gauge) pressure in bar
                                                                                                                   // Relative (gauge) pressure in psi
                                                                                                                   // Relative (gauge) pressure in mega pascal
                                                                                                                   // Relative (gauge) pressure in kilo pascal
                                                                                                   // Absolute pressure in bar
                                                                                       // Absolute pressure in psi
                                                                                       // Absolute pressure in mega pascal
                                                                                       // Absolute pressure in kilo pascal
                                                                                       // Absolute pressure in inches of mercury
                                                                                          // Absolute pressure in inches of water
                                                                                                    // Absolute pressure in milibar


  end PressureSensor;

  partial model FlowSensor
    extends Sensors.FlowSensor                         annotation(IconMap(primitivesVisible=true));


    // Sensor signal parameters
    parameter String signal_unit = "kg/s" "Specify the signal unit. This should be the unit of Q_start and of the tag linked to the sensor." annotation(choices(choice="kg/s", choice="m3/s", choice="l/m", choice="t/h", choice="lb/s", choice="Mlb/h", choice="klb/h"),
    Dialog(tab="General", group="Sensor signal parameters"));

    Utilities.Interfaces.GenericReal      Q_sensor(start=Q_start) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation
    if signal_unit == "l/m" then
      Q_sensor = Q_lm;
    elseif signal_unit == "t/h" then
      Q_sensor = Q_th;
    elseif signal_unit == "lb/s" then
      Q_sensor = Q_lbs;
    elseif signal_unit == "Mlb/h" then
      Q_sensor = Q_Mlbh;
    elseif signal_unit == "klb/h" then
      Q_sensor = Q_klbh;
    elseif signal_unit == "lb/h" then
      Q_sensor = Q_lbh;
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

  partial model DeltaPressureSensor
    extends MetroscopeModelingLibrary.Partial.Sensors.DeltaPressureSensor;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;
    replaceable package Medium =
        MetroscopeModelingLibrary.Partial.Media.PartialMedium;


    parameter String signal_unit = "bar" "Specify the signal unit. This should be the unit of DP_start and of the tag linked to the sensor." annotation(choices(choice="bar", choice="mbar", choice="psi", choice="kPa", choice="Pa", choice="inH2O"),
    Dialog(tab="General", group="Sensor signal parameters"));


    replaceable Partial.Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    replaceable Partial.Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Utilities.Interfaces.GenericReal      DP_sensor(start=DP_start) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation

    if signal_unit == "bar" then
      DP_sensor = DP_bar;
    elseif signal_unit == "mbar" then
      DP_sensor = DP_mbar;
    elseif signal_unit == "psi" then
      DP_sensor = DP_psi;
    elseif signal_unit == "inH2O" then
      DP_sensor = DP_inH2O;
    elseif signal_unit == "kPa" then
      DP_sensor = DP_kPa;
    else
      DP_sensor = DP;
    end if;

    annotation (Icon(graphics={Text(
            extent={{-100,-160},{102,-200}},
            textColor={0,0,0},
            textString=if display_output then
                       if display_unit == "mbar" then DynamicSelect("",String(DP_mbar)+" mbar")
                       else if display_unit == "psi" then DynamicSelect("",String(DP_psi)+" psi")
                       else if display_unit == "kPa" then DynamicSelect("", String(DP_kPa) + " kPa")
                       else if display_unit == "Pa" then DynamicSelect("",String(DP)+" Pa")
                       else DynamicSelect("",String(DP_bar)+" bar")
                       else ""),
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              pattern=LinePattern.None,
              fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
              fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
            Text(
              extent={{-100,160},{100,120}},
              textColor={85,170,255},
              textString="%name"),
            Text(
              extent={{-100,-120},{100,-160}},
              textColor={107,175,17},
              textString=if show_causality then "%causality" else ""),
            Line(
              points={{100,-60},{140,-60},{140,-140},{100,-140}},
              color={107,175,17},
              arrow=if causality == "" or show_causality == false then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
              thickness=0.5,
              pattern=if causality == "" or show_causality == false then LinePattern.None else LinePattern.Solid,
              smooth=Smooth.Bezier)}));
  end DeltaPressureSensor;
annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          fillColor={245,245,245},
          fillPattern=FillPattern.Solid,
          extent={{-56,-56},{56,56}}),
        Line(points={{0,56},{0,26}}),
        Line(points={{28,8},{48.2,29.3}}),
        Line(points={{-28,10},{-48.2,29.3}}),
        Ellipse(
          lineColor={64,64,64},
          fillColor={255,255,255},
          extent={{-12,-50},{12,-26}}),
        Polygon(
          rotation=-17.5,
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-5.0,0.0},{-2.0,60.0},{0.0,65.0},{2.0,60.0},{5.0,0.0}},
        origin={2,-34}),
        Ellipse(
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-7,-45},{7,-31}}),
      Rectangle(
        extent={{-61.1127,14.0711},{61.1127,-14.0711}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45,
        origin={2.83704,2.73655})}));
end SensorsControl;
