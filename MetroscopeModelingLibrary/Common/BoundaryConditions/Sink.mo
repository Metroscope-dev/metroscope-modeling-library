within MetroscopeModelingLibrary.Common.BoundaryConditions;
model Sink
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  Modelica.Units.SI.MassFlowRate Q_in(start=200);
  Modelica.Units.SI.VolumeFlowRate Qv_in(start=200);
  Modelica.Units.SI.AbsolutePressure P_in(start=60e5);
  Modelica.Units.SI.Temperature T_in(start=293.15);
  Modelica.Units.SI.Temperature T_vol(start=293.15);
  Modelica.Units.SI.SpecificEnthalpy h_in(start=3.2e6);
  Modelica.Units.SI.SpecificEnthalpy h_vol(start=3.2e6);
  Modelica.Units.SI.MassFlowRate Qi_in[Medium.nXi];
      Medium.MassFraction Xi_vol[Medium.nXi];
      Medium.MassFraction Xi_in[Medium.nXi];
  PartialBoundaryCondition partialBoundaryCondition(redeclare package                Medium =
        Medium)
    annotation (Placement(transformation(extent={{56,-10},{36,10}})));
  Partial.BasicTransportModel basicTransport(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-28,-10},{-8,10}})));
  Connectors.FluidInlet C_in(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  basicTransport.Q_in + basicTransport.Q_out = 0;
  basicTransport.P_in = basicTransport.P_out;
  basicTransport.Q_in*basicTransport.h_in = - basicTransport.Q_out*basicTransport.h_out;
  basicTransport.Q_in*basicTransport.Xi_in = - basicTransport.Q_out*basicTransport.Xi_out;
  partialBoundaryCondition.Q = Q_in;
  partialBoundaryCondition.P=P_in;
  partialBoundaryCondition.T=T_in;
  partialBoundaryCondition.T_vol=T_vol;
  partialBoundaryCondition.h=h_in;
  partialBoundaryCondition.h_vol=h_vol;
  partialBoundaryCondition.Qi=Qi_in;
  partialBoundaryCondition.Xi_vol=Xi_vol;
  partialBoundaryCondition.Xi=Xi_in;
  Qv_in=basicTransport.Qv_in;
  connect(basicTransport.C_out, partialBoundaryCondition.C)
    annotation (Line(points={{-7.8,0},{36,0}}, color={238,46,47}));
  connect(basicTransport.C_in, C_in)
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
