within MetroscopeModelingLibrary.Partial.Machines;
partial model FixedSpeedPump
  extends BaseClasses.FlowModel(P_out_0=10e5) annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Utilities.Icons.Machines.PumpIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  Unit.Yield rh "Hydraulic efficiency"; // Function of Qv
  Units.Height hn(start=10) "Pump head"; // Function of Qv

  Units.Power Wh "Hydraulic power";

equation

  // Outlet variation
  DP = rho*Constants.g*hn;
  DH = Constants.g *hn/rh;

  // Hydraulic power
  Wh = Qv * DP / rh; // = Qv*rho * g*hn/rh = Q * DH = W
end FixedSpeedPump;
