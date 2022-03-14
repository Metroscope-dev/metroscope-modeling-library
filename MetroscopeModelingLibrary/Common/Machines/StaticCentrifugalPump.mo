within MetroscopeModelingLibrary.Common.Machines;
model StaticCentrifugalPump "Static centrifugal pump"
  extends MetroscopeModelingLibrary.Common.Partial.FlowModel(Q_in_0=500);
  parameter Real rh_0 = 0.85 "Nominal Hydraulic efficiency";
  parameter Real rm_0 = 0.85 "Nominal mechanical efficiency";
  parameter Real Qv_0 = Q_in_0/1000 "Nominal volumic flow rate"; // Qv = Q/rho
  parameter Real hn_0 = 10 "Nominal pump head";

  parameter Boolean adiabatic_compression=false
    "true: compression at constant enthalpy - false: compression with varying enthalpy";

  connector InputReal = input Real;
  connector InputMassFlowRate = input Modelica.Units.SI.MassFlowRate;

  MetroscopeModelingLibrary.Common.Units.AngularVelocity_rpm VRotn(start=1400)
    "Nominal rotational speed";
  InputReal rm(start=rm_0)
    "Product of the pump mechanical and electrical efficiencies";
  InputReal a1(start=-88.67)
    "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  InputReal a2(start=0)
    "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  InputReal a3(start=43.15)
    "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  InputReal b1(start=-3.7751)
    "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  InputReal b2(start=3.61)
    "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  InputReal b3(start=-0.0075464)
    "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";
  InputReal rhmin(start=0.20) "Minimum efficiency to avoid zero crossings";
protected
  constant Modelica.Units.SI.Acceleration g=Modelica.Constants.g_n
    "Gravity constant";
public
  Real rh(nominal=rh_0) "Hydraulic efficiency";
  Modelica.Units.SI.Height hn(start=hn_0, nominal=hn_0) "Pump head";
  Real R(start=1) "Reduced rotational speed";
  //InputMassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv(start=Qv_0, nominal=Qv_0) "Volume flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv0(start=Qv_0, nominal=Qv_0) "Volume flow rate";
  Modelica.Units.SI.Power Wh "Hydraulic power";
  Modelica.Units.SI.Power Wm "Mechanical power";
  Modelica.Units.SI.SpecificEnthalpy DH
    "Specific enthalpy variation between the outlet and the inlet";
   //MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP "Singular pressure loss";
  Modelica.Blocks.Interfaces.RealInput VRot annotation (Placement(
        transformation(extent={{0,-142},{40,-102}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Electrical.Connectors.C_power C_power "Electrical alimentation of the pump" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,112})));
equation
  DH = h_out - h_in;
  //DP = P_out - P_in;
  DP = homotopy(rhom*g*hn, rhom*g*hn_0);
  //DP = rhom*g*hn;
  //Q_in + Q_out = 0; //FlowModel
  //Q = Q_in; //FlowModel
  Q = homotopy(Qv*rhom, Qv_0*rhom);
  //Q = Qv*rhom;
  Q = max(Qv0*rhom, Qeps);

  /* Energy balance equation */
  if adiabatic_compression then
    DH = 0;
  else
    //DH = homotopy(g*hn/rh, g*hn/rh_0);
    DH = g*hn/rh;
  end if;

  /* Reduced rotational speed */
  R = VRot/VRotn;

  /* Pump characteristics */
  hn = noEvent(homotopy(a1*Qv0*abs(Qv0), a1*Qv_0*abs(Qv_0)) + a2*Qv0*R + a3*R^2); // NON LINEAR
  //hn = noEvent(a1*Qv0*abs(Qv0) + a2*Qv0*R + a3*R^2); // NON LINEAR
  rh = noEvent(max(if (abs(R) > eps) then b1*Qv*abs(Qv)/R^2 + b2*Qv/R + b3 else b3, rhmin)); // NON LINEAR

  /* Mechanical power */
  Wm + C_power.W = 0; // C_power.W is negative since it is power fed to the component
  Wm = homotopy(DH*Q/rm, DH*Q_in_0/rm_0); // Wm is positive since it is the power produced by the pump
  //Wm = DH*Q/rm; // Wm is positive since it is the power produced by the pump
  /* Hydraulic power */
  Wh = homotopy(Qv*DP/rh, Qv_0*DP/rh_0);
  //Wh = Qv*DP/rh;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}),
    Documentation(info="<html>
<p><b>Copyright &copy; EDF 2002 - 2013</b> </p>
<p><b>ThermoSysPro Version 3.1</b> </p>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end StaticCentrifugalPump;
