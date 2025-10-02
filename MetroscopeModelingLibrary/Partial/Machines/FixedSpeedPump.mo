within MetroscopeModelingLibrary.Partial.Machines;
partial model FixedSpeedPump
  extends BaseClasses.FlowModel(P_out_0=10e5) annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Utilities.Icons.Machines.PumpIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  Utilities.Interfaces.GenericReal rh annotation (Placement(transformation(extent={{-96,50},{-76,70}}), iconTransformation(extent={{-80,20},
            {-120,60}})));
  Utilities.Interfaces.GenericReal hn annotation (Placement(transformation(extent={{-74,70},{-54,90}}), iconTransformation(extent={{-58,60},
            {-98,100}})));
equation

  // Outlet variation
  DP = rho*Constants.g*hn;
  DH = Constants.g *hn/rh;
end FixedSpeedPump;
