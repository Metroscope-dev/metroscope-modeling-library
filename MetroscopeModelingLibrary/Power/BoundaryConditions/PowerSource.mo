within MetroscopeModelingLibrary.Power.BoundaryConditions;
model PowerSource
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.PowerSourceIcon;

  Units.OutletPower W_out;
  Connectors.PowerOutlet C_W_out annotation (Placement(transformation(extent={{38,-10},{58,10}}), iconTransformation(extent={{38,-10},{58,10}})));
equation
  W_out = C_W_out.W;
end PowerSource;
