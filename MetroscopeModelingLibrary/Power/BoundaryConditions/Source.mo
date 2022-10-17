within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSourceIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputNotUsed dummy; // To keep local balance of the Power.Sink, since we cannot define 'dummy = 0' here
  Units.NegativePower W_out;
  Connectors.Outlet C_out(dummy = dummy) annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_out.W;
end Source;
