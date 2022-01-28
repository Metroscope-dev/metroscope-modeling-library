within MetroscopeModelingLibrary.Common.BoundaryConditions;
model Sink
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;

  connector InputAbsolutePressure = input Modelica.Units.SI.AbsolutePressure;
  connector InputSpecificEnthalpy = input Modelica.Units.SI.SpecificEnthalpy;
  connector InputMassFraction = input Medium.MassFraction;

  Modelica.Units.SI.MassFlowRate Q_in(start=500);
  Modelica.Units.SI.VolumeFlowRate Qv_in(start=500);
  InputAbsolutePressure P_in(start=1e5);
  Modelica.Units.SI.Temperature T_in(start=293.15);
  Modelica.Units.SI.SpecificEnthalpy h_in(start=1e5);
  Medium.MassFraction Xi_in[Medium.nXi];

  PartialBoundaryCondition partialBoundaryCondition(redeclare package                Medium =
        Medium)
    annotation (Placement(transformation(extent={{56,-10},{36,10}})));
  Partial.IsoPIsoHFlowModel flowModel(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-28,-10},{-8,10}})));
  Connectors.FluidInlet C_in(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  partialBoundaryCondition.Q = Q_in;
  partialBoundaryCondition.P=P_in;
  partialBoundaryCondition.T=T_in;
  partialBoundaryCondition.h=h_in;
  inStream(partialBoundaryCondition.C.h_outflow)=partialBoundaryCondition.h;
  partialBoundaryCondition.C.h_outflow = 1e5;
  partialBoundaryCondition.Xi=Xi_in;
  inStream(partialBoundaryCondition.C.Xi_outflow)=partialBoundaryCondition.Xi;
  partialBoundaryCondition.C.Xi_outflow = zeros(Medium.nXi);
  Qv_in=flowModel.Qv_in;
  connect(flowModel.C_out, partialBoundaryCondition.C)
    annotation (Line(points={{-7.8,0},{36,0}}, color={238,46,47}));
  connect(flowModel.C_in, C_in)
    annotation (Line(points={{-28,0},{-100,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-98,0},{-60,0},{-74,10}}),
        Line(points={{-74,-10},{-60,0}}),
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-50,50},{50,-50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end Sink;
