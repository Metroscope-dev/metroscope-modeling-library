within MetroscopeModelingLibrary.Common.BoundaryConditions;
model PartialBoundaryCondition
  replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  extends MetroscopeModelingLibrary.Common.Constants.Constants;

  connector InputAbsolutePressure = input Modelica.Units.SI.AbsolutePressure;
  connector InputSpecificEnthalpy = input Modelica.Units.SI.SpecificEnthalpy;
  connector InputMassFraction = input Medium.MassFraction;

  Modelica.Units.SI.MassFlowRate Q(start=500);
  InputAbsolutePressure P(start=1e5);
  Modelica.Units.SI.Temperature T(start=293.15);
  Modelica.Units.SI.SpecificEnthalpy h(start=1e5);
  Medium.MassFraction Xi[Medium.nXi];
  Medium.ThermodynamicState state;
  Connectors.FluidPort C(redeclare package Medium = Medium) annotation (
      Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(
          extent={{90,-10},{110,10}})));
equation
  P =C.P;
  Q =C.Q;
  state = Medium.setState_phX(P,h,Xi);
  T = Medium.temperature(state);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{40,0},{90,0},{72,10}}),
        Rectangle(
          extent={{-40,40},{40,-40}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{90,0},{72,-10}})}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end PartialBoundaryCondition;
