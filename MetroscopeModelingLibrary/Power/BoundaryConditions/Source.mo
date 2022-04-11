within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSourceIcon;

  Units.OutletPower W_out;
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_out.W;
end Source;
