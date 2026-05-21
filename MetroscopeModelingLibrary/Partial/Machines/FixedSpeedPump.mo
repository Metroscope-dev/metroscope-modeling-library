within MetroscopeModelingLibrary.Partial.Machines;
partial model FixedSpeedPump
  extends BaseClasses.FlowModel(P_out_0=10e5) annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Utilities.Icons.Machines.PumpIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  Utilities.Interfaces.GenericReal rh annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,-80}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,-80})));
  Utilities.Interfaces.GenericReal hn annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-80}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-80})));
equation

  // Outlet variation
  DP = rho*Constants.g*hn;
  DH = Constants.g *hn/rh;
end FixedSpeedPump;
