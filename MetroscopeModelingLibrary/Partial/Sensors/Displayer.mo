within MetroscopeModelingLibrary.Partial.Sensors;
partial model Displayer

  import MetroscopeModelingLibrary.Utilities.Units;
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  // Initialization parameters
  parameter Units.PositiveMassFlowRate Q_0 = 100;
  parameter Units.Pressure P_0 = 1e5;
  parameter Units.SpecificEnthalpy h_0 = 5e5;


  replaceable BaseClasses.IsoPHFlowModel flow_model annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  replaceable Connectors.FluidInlet C_in(Q(start=Q_0, nominal=Q_0), P(start=P_0, nominal=P_0), redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-40,-10},{-20,10}}), iconTransformation(extent={{-40,-10},{-20,10}})));
  replaceable Connectors.FluidOutlet C_out(Q(start=-Q_0, nominal=Q_0), P(start=P_0, nominal=P_0), redeclare package Medium = Medium) annotation (Placement(transformation(extent={{20,-10},{40,10}}), iconTransformation(extent={{20,-10},{40,10}})));

equation
  connect(flow_model.C_in, C_in) annotation (Line(points={{-10,0},{-30,0}}, color={95,95,95}));
  connect(flow_model.C_out, C_out) annotation (Line(points={{10,0},{30,0}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
                   graphics={
      Text(
        extent={{-80,60},{80,40}},
        textColor={0,0,0},
          textString=DynamicSelect("",String(flow_model.T-273.15)+" degC")),
      Text(
        extent={{-80,82},{80,62}},
        textColor={0,0,0},
          textString=DynamicSelect("",String(flow_model.P*1e-5)+" barA")),
      Text(
        extent={{-80,100},{80,80}},
        textColor={0,0,0},
          textString=DynamicSelect("",String(flow_model.Q)+" kg/s")),
        Line(
          points={{0,40},{0,0}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{20,20},{0,0},{-20,20}},
          color={238,46,47},
          thickness=1),
      Text(
        extent={{-80,120},{80,100}},
        textColor={0,0,0},
          textString=DynamicSelect("",String(flow_model.h*1e-6)+" MJ/kg"))}), Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end Displayer;
