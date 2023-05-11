within MetroscopeModelingLibrary.Partial.Machines;
partial model Pump
  extends BaseClasses.FlowModel(P_out_0=10e5) annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Utilities.Icons.Machines.PumpIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;


  Real VRotn(start=1400, min=0, nominal=2000) "Nominal rotational speed";
  Inputs.InputReal a1(start=0) "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Inputs.InputReal a2(start=0) "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Inputs.InputHeight a3(start=10) "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Inputs.InputReal b1(start=0) "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Inputs.InputReal b2(start=0) "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Inputs.InputYield b3(start=0.8) "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";

  Inputs.InputYield rm(start=0.85) "Product of the pump mechanical and electrical efficiencies";
  Inputs.InputYield rh_min(start=0.20) "Minimum efficiency to avoid zero crossings";

  Units.Yield rh "Hydraulic efficiency";
  Units.Height hn(start=10) "Pump head";
  Units.Fraction R(start=1) "Reduced rotational speed";

  Units.Power Wh "Hydraulic power";
  Units.PositivePower Wm "Mechanical power";

  Modelica.Blocks.Interfaces.RealInput VRot "Pump rotational speed" annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-98}),                             iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Power.Connectors.Inlet C_power "Electrical alimentation of the pump" annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={0,108}), iconTransformation(
        extent={{-12,-12},{12,12}},
        rotation=-90,
        origin={0,108})));
equation
  // internal variables
  R = VRot/VRotn; // Reduced rotational speed

  // Pump characteristics
  hn = a1*Qv^2 + a2*Qv*R + a3*R^2;
  rh =noEvent(max(if (R > 1e-5) then b1*Qv^2/R^2 + b2*Qv/R + b3 else b3, rh_min));

  // Outlet variation
  DP = rho*Constants.g*hn;
  DH = Constants.g *hn/rh;

  // Mechanical power
  Wm = C_power.W; // C_power.W is positive since it is power fed to the component
  Wm = W/rm; // Wm is positive since it is the power produced by the pump

  // Hydraulic power
  Wh = Qv * DP / rh; // = Qv*rho * g*hn/rh = Q * DH = W
end Pump;
