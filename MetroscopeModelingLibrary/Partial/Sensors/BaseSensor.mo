within MetroscopeModelingLibrary.Partial.Sensors;
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
  replaceable BaseClasses.IsoPHFlowModel flow_model(P_0=P_0, Q_0=Q_0, h_0=h_0) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
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
