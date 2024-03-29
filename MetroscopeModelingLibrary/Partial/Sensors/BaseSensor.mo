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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end BaseSensor;
