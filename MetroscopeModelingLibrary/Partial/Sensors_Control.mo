within MetroscopeModelingLibrary.Partial;
package Sensors_Control
  partial model BaseSensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;

    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Utilities.Units;

    // Initialization parameters
    parameter Units.PositiveMassFlowRate Q_0=100;
    parameter Units.Pressure P_0 = 1e5;
    parameter Units.SpecificEnthalpy h_0 = 5e5;

    // Input Quantity
    Units.PositiveMassFlowRate Q(start=Q_0, nominal=Q_0) "Component mass flow rate";
    Units.MassFraction Xi[Medium.nXi] "Component mass fractions";
    Units.Pressure P(start=P_0) "Pressure of the fluid into the component";
    Units.SpecificEnthalpy h(start=h_0) "Enthalpy of the fluid into the component";
    Medium.ThermodynamicState state;

    // Failure modes
    parameter Boolean faulty_flow_rate = false;
    Units.MassFlowRate mass_flow_rate_bias(start=0); // mass_flow_rate_bias > 0 means that more mass flow enters the component

    // Icon parameters
    parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
      annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
    parameter String causality = "" "Specify which parameter is calibrated by this sensor";
    outer parameter Boolean show_causality = true "Used to show or not the causality";

    replaceable Connectors.FluidInlet C_in(Q(start=Q_0, nominal=Q_0), P(start=P_0, nominal=P_0), redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    replaceable Connectors.FluidOutlet C_out(Q(start=-Q_0, nominal=Q_0), P(start=P_0, nominal=P_0), redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    replaceable BaseClasses.IsoPHFlowModel flow_model annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  equation
    if not faulty_flow_rate then
      mass_flow_rate_bias = 0;
    end if;

    P = C_in.P;
    Q = C_in.Q + mass_flow_rate_bias;
    Xi = inStream(C_in.Xi_outflow);
    h = inStream(C_in.h_outflow);

    state = Medium.setState_phX(P, h, Xi);

    assert(Q > 0, "Wrong flow sign in inline sensor. Common causes : outlet connected as if it was inlet and vice versa, or Positive/NegativeMassflowrate misuse. Recall : inlet flow is positive, outlet is negatve", AssertionLevel.warning);
    connect(flow_model.C_in, C_in) annotation (Line(points={{-10,0},{-100,0}}, color={95,95,95}));
    connect(flow_model.C_out, C_out) annotation (Line(points={{10,0},{100,0}}, color={0,0,0}));
    annotation (Icon(
      graphics={
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
  end BaseSensor;

  partial model TemperatureSensor
    extends BaseSensor                                   annotation(IconMap(primitivesVisible=true));
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Constants;

    // Initialization parameters
    parameter Units.Temperature T_0 = 300;

    Units.Temperature T(start=T_0); // Temperature in SI Units : K
    Real T_degC(unit="degC", start=T_0 +Constants.T0_degC_in_K,  nominal=T_0 +Constants.T0_degC_in_K);   // Temperature in degC
    Real T_degF(unit="degF",
                start=(T_0 +Constants.T0_degC_in_K) *Constants.degC_to_degF +Constants.T0_degC_in_degF,
                nominal=(T_0 +Constants.T0_degC_in_K) *Constants.degC_to_degF +Constants.T0_degC_in_degF);   // Temperature in degF

    parameter String display_unit = "degC" "Specify the display unit"
      annotation(choices(choice="degC", choice="K", choice="degF"));
    outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
    parameter String output_signal_unit = "degC" annotation (choices(choice="degC", choice="K", choice="degF"));

    Modelica.Blocks.Interfaces.RealOutput T_sensor annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation
    T =flow_model.T;
    T_degC +Constants.T0_degC_in_K  = T; // Conversion K to Celsius
    T_degF = T_degC*Constants.degC_to_degF +Constants.T0_degC_in_degF;   // Conversion Celsius to Farenheit

    if output_signal_unit == "degC" then
      T_sensor = T_degC;
    elseif output_signal_unit == "degF" then
      T_sensor = T_degF;
    else
      T_sensor = T;
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
    extends BaseSensor                                   annotation(IconMap(primitivesVisible=true));
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Constants;

    // All nominal values of gauge pressures are set to the absolute pressure value to avoid zero nominal value

    Real P_barG(nominal=P_0, start=P_0*Constants.Pa_to_barA - Constants.atmospheric_pressure_in_bar); // Relative (gauge) pressure in bar

    Real P_psiG(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA - Constants.P0_psiG_in_psiA); // Relative (gauge) pressure in psi
    Real P_MPaG(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA - Constants.P0_MPaG_in_MPaA); // Relative (gauge) pressure in mega pascal
    Real P_kPaG(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA - Constants.P0_kPaG_in_kPaA); // Relative (gauge) pressure in kilo pascal
    Real P_barA(nominal = P_0*Constants.Pa_to_barA, start = P_0*Constants.Pa_to_barA, unit="bar"); // Absolute pressure in bar
    Real P_psiA(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA); // Absolute pressure in psi
    Real P_MPaA(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA); // Absolute pressure in mega pascal
    Real P_kPaA(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA); // Absolute pressure in kilo pascal

    Real P_inHg(nominal = P_0*Constants.Pa_to_inHg, start = P_0*Constants.Pa_to_inHg); // Absolute pressure in inches of mercury
    Real P_mbar(nominal = P_0*Constants.Pa_to_mbar, start = P_0*Constants.Pa_to_mbar, unit="mbar"); // Absolute pressure in milibar

    outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
    parameter String display_unit = "barA" "Specify the display unit"
      annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA"));
    parameter String output_signal_unit = "barA" annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA"));

    Modelica.Blocks.Interfaces.RealOutput P_sensor annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation
    P_barA = P * Constants.Pa_to_barA;
    P_psiA = P * Constants.Pa_to_psiA;
    P_MPaA = P * Constants.Pa_to_MPaA;
    P_kPaA = P * Constants.Pa_to_kPaA;

    P_barG =P_barA - Constants.atmospheric_pressure_in_bar;
    P_psiG = P_psiA - Constants.P0_psiG_in_psiA;
    P_MPaG = P_MPaA - Constants.P0_MPaG_in_MPaA;
    P_kPaG = P_kPaA - Constants.P0_kPaG_in_kPaA;

    P_mbar = P * Constants.Pa_to_mbar;
    P_inHg = P * Constants.Pa_to_inHg;

    if output_signal_unit == "barA" then
      P_sensor = P_barA;
    elseif output_signal_unit == "barG" then
      P_sensor = P_barG;
    elseif output_signal_unit == "mbar" then
      P_sensor = P_mbar;
    elseif output_signal_unit == "MPaA" then
      P_sensor = P_MPaA;
    elseif output_signal_unit == "kPaA" then
      P_sensor = P_kPaA;
    else
      P_sensor = P;
    end if;

    annotation (Icon(graphics={Text(
            extent={{-100,-160},{102,-200}},
            textColor={0,0,0},
            textString=if display_output then
                       if display_unit == "barG" then DynamicSelect("",String(P_barG)+" barG")
                       else if display_unit == "mbar" then DynamicSelect("",String(P_mbar)+" mbar")
                       else if display_unit == "MPaA" then DynamicSelect("",String(P_MPaA)+" MPaA")
                       else if display_unit == "kPaA" then DynamicSelect("",String(P_kPaA)+" kPaA")
                       else DynamicSelect("",String(P_barA)+" barA")
                       else "")}));
  end PressureSensor;

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

    // Failure modes
    parameter Boolean faulty = false;

    parameter String display_unit = "kg/s" "Specify the display unit"
      annotation(choices(choice="kg/s", choice="m3/s", choice="l/m", choice="t/h", choice="lb/s", choice="Mlb/h"));
    outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
    parameter String output_signal_unit = "kg/s";

    Modelica.Blocks.Interfaces.RealOutput Q_sensor annotation (Placement(transformation(
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

    if output_signal_unit == "l/m" then
      Q_sensor = Q_lm;
    elseif output_signal_unit == "t/h" then
      Q_sensor = Q_th;
    else
      Q_sensor = Q;
    end if;

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
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;
    replaceable package Medium =
        MetroscopeModelingLibrary.Partial.Media.PartialMedium;

    parameter Utilities.Units.DifferentialPressure DP_0=1e4;
    Utilities.Units.DifferentialPressure DP(start=DP_0, nominal=DP_0);
    Real DP_bar(unit="bar", start=DP_0*Utilities.Constants.Pa_to_barA); // Pressure difference in bar
    Real DP_mbar(unit="mbar", start=DP_0*Utilities.Constants.Pa_to_mbar); // Pressure difference in mbar
    Real DP_psi(start=DP_0*Utilities.Constants.Pa_to_psiA); // Pressure difference in PSI

    // Icon parameters
    parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
      annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
    parameter String causality = "" "Specify which parameter is calibrated by this sensor";
    outer parameter Boolean show_causality = true "Used to show or not the causality";
    outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
    parameter String display_unit = "bar" "Specify the display unit"
      annotation(choices(choice="bar", choice="mbar", choice="psi", choice="Pa"));
    parameter String output_signal_unit = "bar" annotation(choices(choice="bar", choice="mbar", choice="psi", choice="Pa"));

    replaceable Partial.Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    replaceable Partial.Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Interfaces.RealOutput DP_sensor annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation
    // No inlet except pressure
    C_in.Q = 0;
    C_in.h_outflow = 0;
    C_in.Xi_outflow = zeros(Medium.nXi);

    // No outlet except pressure
    C_out.Q = 0;
    C_out.h_outflow = 0;
    C_out.Xi_outflow = zeros(Medium.nXi);

    // Conversions
    DP = C_out.P - C_in.P;
    DP_bar =DP*Utilities.Constants.Pa_to_barA;
    DP_mbar =DP*Utilities.Constants.Pa_to_mbar;
    DP_psi =DP*Utilities.Constants.Pa_to_psiA;

    if output_signal_unit == "bar" then
      DP_sensor = DP_bar;
    elseif output_signal_unit == "mbar" then
      DP_sensor = DP_mbar;
    elseif output_signal_unit == "psi" then
      DP_sensor = DP_psi;
    else
      DP_sensor = DP;
    end if;

    annotation (Icon(graphics={Text(
            extent={{-100,-160},{102,-200}},
            textColor={0,0,0},
            textString=if display_output then
                       if display_unit == "mbar" then DynamicSelect("",String(DP_mbar)+" mbar")
                       else if display_unit == "psi" then DynamicSelect("",String(DP_psi)+" psi")
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
end Sensors_Control;
