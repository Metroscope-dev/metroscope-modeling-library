within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model PartialBoundaryCondition
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  Units.Pressure P(start=1e5);
  Units.MassFlowRate Q(start=500);
  Units.Temperature T(start=300);
  Units.SpecificEnthalpy h(start=1e5);
  Units.MassFraction Xi[Medium.nXi];
  Medium.ThermodynamicState state;

  replaceable MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidPort C(redeclare package Medium = Medium);
equation
  // Connector
  P = C.P;
  Q = C.Q;

  // State
  state = Medium.setState_phX(P,h,Xi);

  // Computed quantity
  T = Medium.temperature(state);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end PartialBoundaryCondition;
