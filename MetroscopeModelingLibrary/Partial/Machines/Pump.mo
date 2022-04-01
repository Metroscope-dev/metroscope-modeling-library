within MetroscopeModelingLibrary.Partial.Machines;
partial model Pump
  extends BaseClasses.FlowModel;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;

  MetroscopeModelingLibrary.Common.Units.AngularVelocity_rpm VRotn(start=1400) "Nominal rotational speed";
  Inputs.InputReal a1(start=-88.67) "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Inputs.InputReal a2(start=0) "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Inputs.InputReal a3(start=43.15) "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Inputs.InputReal b1(start=-3.7751) "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Inputs.InputReal b2(start=3.61) "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Inputs.InputReal b3(start=-0.0075464) "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";

  Inputs.InputYield rm(start=0.85) "Product of the pump mechanical and electrical efficiencies";
  Inputs.InputYield rhmin(start=0.20) "Minimum efficiency to avoid zero crossings";

  Units.Yield rh "Hydraulic efficiency";
  Units.Height hn(start=10) "Pump head";
  Units.Fraction R(start=1) "Reduced rotational speed";
  InputMassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv(start=0.5) "Volume flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv0(start=0.5) "Volume flow rate";
  Modelica.Units.SI.Power Wh "Hydraulic power";
  Modelica.Units.SI.Power Wm "Mechanical power";
  Modelica.Units.SI.SpecificEnthalpy deltaH
    "Specific enthalpy variation between the outlet and the inlet";
   MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP "Singular pressure loss";
  Modelica.Blocks.Interfaces.RealInput VRot annotation (Placement(
        transformation(extent={{0,-142},{40,-102}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Electrical.Connectors.C_power C_power "Electrical alimentation of the pump" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,112})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Pump;
