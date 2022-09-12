within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSinkIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Units.PositivePower W_in;
  Connectors.Inlet C_in(not_used = 0) annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  W_in = C_in.W;
end Sink;
