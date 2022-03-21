within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model PartialBoundaryCondition
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  Units.Temperature T(start=300);
  Units.SpecificEnthalpy h(start=1e5);
  Units.MassFraction Xi[Medium.nXi];
  Medium.ThermodynamicState state;

  // To be set by connector
  Units.Pressure P(start=1e5);
  Units.MassFlowRate Q(start=500);
  Units.VolumeFlowRate Qv(start=0.1);

  replaceable MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidPort C;
equation
  // Connector
  C.P = P;
  C.Q = Q;

  // State
  state = Medium.setState_phX(P,h,Xi);

  // Computed quantity
  T = Medium.temperature(state);
  Qv = Q / Medium.density(state);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1)}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end PartialBoundaryCondition;
