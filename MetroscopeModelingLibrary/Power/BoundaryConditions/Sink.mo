within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSinkIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputNotUsed not_used; // To keep local balance of the Power.Sink, since we cannot define 'not_used = 0' here
  Units.PositivePower W_in;
  Connectors.Inlet C_in(not_used = not_used) annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  W_in = C_in.W;
end Sink;
