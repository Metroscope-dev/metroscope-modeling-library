within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model PartialBoundaryCondition
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Input Quantities
  Inputs.InputSpecificEnthalpy h(start=1e5);
  Inputs.InputPressure P(start=1e5);
  Units.MassFlowRate Q(start=500);
  Inputs.InputMassFraction Xi[Medium.nXi];

  Units.VolumeFlowRate Qv(start=0.5);

  // Computed quantities
  Units.Temperature T(start=300);
  Medium.ThermodynamicState state;

  replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidPort C
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}), iconTransformation(extent={{-10,-10},{10,10}})));
equation
  // Connector
  C.P = P;
  C.Q = Q;

  // State
  state = Medium.setState_phX(P,h,Xi);

  // Computed quantities
  T = Medium.temperature(state);
  Qv = Q / Medium.density(state);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
                                          Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end PartialBoundaryCondition;
