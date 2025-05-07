within MetroscopeModelingLibrary.Partial.Machines;
partial model Pump
  extends MetroscopeModelingLibrary.Partial.Machines.FixedSpeedPump;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  Inputs.InputReal rm(start=0.85);
  Real VRotn(start=1400, min=0, nominal=2000) "Nominal rotational speed";
  Inputs.InputReal a1(start=0) "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Inputs.InputReal a2(start=0) "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Inputs.InputHeight a3(start=10) "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Inputs.InputReal b1(start=0) "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Inputs.InputReal b2(start=0) "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Inputs.InputReal b3(start=0.8) "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";

  Inputs.InputYield rh_min(start=0.20) "Minimum efficiency to avoid zero crossings";

  Units.Fraction R(start=1) "Reduced rotational speed";

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

  C_power.W = W/rm;
  annotation (Icon(graphics={
        Line(
          points={{-100,-100},{100,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{90,70},{100,100}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{70,90},{100,100}},
          color={0,0,0},
          thickness=0.5)}));
end Pump;
