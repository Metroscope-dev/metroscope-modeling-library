within MetroscopeModelingLibrary.Partial.Machines;
partial model Pump
  extends BaseClasses.FlowModel(P_out_0=10e5) annotation(IconMap(primitivesVisible=false));
  extends MetroscopeModelingLibrary.Icons.Machines.PumpIcon;

  parameter Boolean adiabatic_compression=false
    "true: compression at constant enthalpy - false: compression with varying enthalpy";

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;

  Units.PositiveVolumeFlowRate Qv_in(start=1);

  Real VRotn(start=1400, min=0, nominal=2000) "Nominal rotational speed";
  Inputs.InputReal a1(start=0) "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Inputs.InputReal a2(start=0) "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Inputs.InputHeight a3(start=10) "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Inputs.InputReal b1(start=0) "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Inputs.InputReal b2(start=0) "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Inputs.InputYield b3(start=0.8) "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";

  Inputs.InputYield rm(start=0.85) "Product of the pump mechanical and electrical efficiencies";
  Inputs.InputYield rhmin(start=0.20) "Minimum efficiency to avoid zero crossings";

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
  Qv_in = Q / Medium.density(state_in);
  DP = rho*Constants.g*hn;

  if adiabatic_compression then
    W = 0;
  else
    h_out-h_in = Constants.g*hn/rh;
  end if;

  // Reduced rotational speed
  R = VRot/VRotn;

  // Pump characteristics
  hn = a1*Qv_in^2 + a2*Qv_in*R + a3*R^2;
  rh = noEvent(max(if (R > 1e-5) then b1*Qv_in^2/R^2 + b2*Qv_in/R + b3 else b3, rhmin));

  // Mechanical power
  Wm = C_power.W; // C_power.W is positive since it is power fed to the component
  Wm = W/rm; // Wm is positive since it is the power produced by the pump

  // Hydraulic power
  Wh = Qv_in * DP / rh;
end Pump;
