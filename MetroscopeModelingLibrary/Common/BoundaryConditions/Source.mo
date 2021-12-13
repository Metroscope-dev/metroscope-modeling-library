within MetroscopeModelingLibrary.Common.BoundaryConditions;
model Source
  replaceable package Medium =
    MetroscopeModelingLibrary.Common.Medium.PartialMedium;

  connector InputAbsolutePressure = input Modelica.Units.SI.AbsolutePressure;
  connector InputSpecificEnthalpy = input Modelica.Units.SI.SpecificEnthalpy;
  connector InputMassFraction = input Medium.MassFraction;

  Modelica.Units.SI.MassFlowRate Q_out(start=500);
  Modelica.Units.SI.VolumeFlowRate Qv_out(start=500);
  InputAbsolutePressure P_out(start=1e5);
  Modelica.Units.SI.Temperature T_out(start=293.15);
  Modelica.Units.SI.Temperature T_vol(start=293.15);
  Modelica.Units.SI.SpecificEnthalpy h_out(start=1e5);
  InputSpecificEnthalpy h_vol(start=1e5);
  Modelica.Units.SI.MassFlowRate Qi_out[Medium.nXi];
  InputMassFraction Xi_vol[Medium.nXi];
    Medium.MassFraction Xi_out[Medium.nXi];
PartialBoundaryCondition partialBoundaryCondition(redeclare package                Medium =
      Medium)
  annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
Partial.BasicTransportModel basicTransport(redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{-28,-10},{-8,10}})));
Connectors.FluidOutlet C_out(redeclare package                Medium =
      Medium)
  annotation (Placement(transformation(extent={{90,-10},{110,10}})));
equation
basicTransport.Q_in + basicTransport.Q_out = 0;
  basicTransport.P_in = basicTransport.P_out;
  basicTransport.Q_in*basicTransport.h_in = - basicTransport.Q_out*basicTransport.h_out;
  basicTransport.Q_in*basicTransport.Xi_in = - basicTransport.Q_out*basicTransport.Xi_out;
partialBoundaryCondition.Q = Q_out;
partialBoundaryCondition.P=P_out;
partialBoundaryCondition.T=T_out;
partialBoundaryCondition.T_vol=T_vol;
partialBoundaryCondition.h=h_out;
partialBoundaryCondition.h_vol=h_vol;
partialBoundaryCondition.Qi=Qi_out;
partialBoundaryCondition.Xi_vol=Xi_vol;
partialBoundaryCondition.Xi=Xi_out;
Qv_out=basicTransport.Qv_out;
connect(partialBoundaryCondition.C, basicTransport.C_in)
  annotation (Line(points={{-48,0},{-28,0}}, color={0,0,0}));
connect(basicTransport.C_out, C_out)
  annotation (Line(points={{-7.8,0},{100,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{40,0},{90,0},{72,10}}),
        Line(points={{90,0},{72,-10}}),
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0})}),           Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end Source;
