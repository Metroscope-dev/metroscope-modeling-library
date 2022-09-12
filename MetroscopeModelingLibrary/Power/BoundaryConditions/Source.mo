within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSourceIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Units.NegativePower W_out;
  Connectors.Outlet C_out(not_used = 0) annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_out.W;
end Source;
