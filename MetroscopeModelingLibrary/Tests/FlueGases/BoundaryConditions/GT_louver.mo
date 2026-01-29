within MetroscopeModelingLibrary.Tests.FlueGases.BoundaryConditions;
model GT_louver
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;


  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{66,-20},{106,20}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.GT_louvers gT_louvers annotation (Placement(transformation(extent={{-40,-40},{40,40}})));
  Utilities.Interfaces.RealInput u(start=10) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-16,80}), iconTransformation(extent={{-358,-90},{-318,-50}})));
  Utilities.Interfaces.RealInput P(start=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,80}), iconTransformation(extent={{-358,-90},{-318,-50}})));
  Utilities.Interfaces.RealInput H(start=50) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={16,80}), iconTransformation(extent={{-358,-90},{-318,-50}})));
  Sensors_Control.FlueGases.FlowSensor flowSensor annotation (Placement(transformation(extent={{42,-10},{62,10}})));
  Utilities.Interfaces.RealInput Q(start=50) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={52,36}), iconTransformation(extent={{-358,-90},{-318,-50}})));
equation


  connect(gT_louvers.T, u) annotation (Line(points={{-16,40},{-16,80}}, color={0,0,127}));
  connect(gT_louvers.P, P) annotation (Line(points={{0,40},{0,80}}, color={0,0,127}));
  connect(gT_louvers.H, H) annotation (Line(points={{16,40},{16,80}}, color={0,0,127}));
  connect(sink.C_in, flowSensor.C_out) annotation (Line(points={{76,0},{62,0}}, color={95,95,95}));
  connect(flowSensor.C_in, gT_louvers.C_out) annotation (Line(points={{42,0},{24,0}}, color={95,95,95}));
  connect(flowSensor.Q_sensor, Q) annotation (Line(points={{52,10},{52,36}}, color={0,0,127}));
end GT_louver;
