within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportQ
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  // ------ Initialization parameters ------
  parameter Units.MassFlowRate Q_in_0 = 100;
  parameter Units.MassFlowRate Q_out_0 = - Q_in_0;

  // ------ Input Quantities ------
  Units.MassFlowRate Q_in(start=Q_in_0) "Inlet Mass flow rate";
  Units.MassFlowRate Q_out(start=Q_out_0) "Outlet Mass flow rate";
  Units.MassFlowRate Qm(start=Qm_0) "Mean Mass flow rate";

  // Connectors
  Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-44,-12},{-24,8}})));
  Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{26,-12},{46,8}})));
protected
  parameter Units.MassFlowRate Qm_0 = (Q_in_0 + Q_out_0)/2;
equation
  Q_in = C_in.Q;
  Q_out = C_out.Q;
  Qm = (Q_in-Q_out)/2;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),                                   Rectangle(
          extent={{-34,28},{36,-32}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}),                                          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PartialTransportQ;
