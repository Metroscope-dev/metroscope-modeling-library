within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSinkIcon;

  Units.PositivePower W_in;
  Connectors.Inlet C_in annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  W_in = C_in.W;
end Sink;
