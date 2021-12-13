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
  Modelica.Units.SI.Temperature T_vol(start=293.15);
  Modelica.Units.SI.SpecificEnthalpy h(start=1e5);
  InputSpecificEnthalpy h_vol(start=1e5);
  Modelica.Units.SI.MassFlowRate Qi[Medium.nXi];
  InputMassFraction Xi_vol[Medium.nXi];
  Medium.MassFraction Xi[Medium.nXi];
  Medium.ThermodynamicState state;
  Medium.ThermodynamicState state_vol;
  Connectors.FluidPort C(redeclare package Medium = Medium) annotation (
      Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(
          extent={{90,-10},{110,10}})));
equation
  P =C.P;
  Q =C.Q;
  Qi =C.Qi;
  C.H = Q*h;
  C.Qi = Q*Xi;
  C.h_vol = h_vol;
  C.Xi_vol = Xi_vol;
  state_vol = Medium.setState_phX(P, h_vol, Xi_vol);
  state = Medium.setState_phX(P, h,Xi);
  T = Medium.temperature(state);
  T_vol = Medium.temperature(state_vol);
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
