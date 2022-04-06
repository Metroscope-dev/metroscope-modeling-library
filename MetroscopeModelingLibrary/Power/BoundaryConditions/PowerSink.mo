within MetroscopeModelingLibrary.Power.BoundaryConditions;
model PowerSink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSinkIcon;

  Units.InletPower W_in;
  Connectors.PowerInlet C_in annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  W_in = C_in.W;
end PowerSink;
