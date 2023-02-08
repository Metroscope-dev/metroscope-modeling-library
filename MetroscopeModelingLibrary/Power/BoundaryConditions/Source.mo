within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.BoundaryConditions.PowerSourceIcon;

  Utilities.Units.NegativePower W_out;
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_out.W;
end Source;
