within MetroscopeModelingLibrary.Common.Machines;
model StaticCentrifugalPump "Static centrifugal pump"
  extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;
  parameter Boolean adiabatic_compression=false
    "true: compression at constant enthalpy - false: compression with varying enthalpy";
public
  MetroscopeModelingLibrary.Common.Units.AngularVelocity_rpm VRotn(start=1400)
    "Nominal rotational speed";
  Real rm(start=0.85)
    "Product of the pump mechanical and electrical efficiencies";
  Real a1(start=-88.67)
    "x^2 coef. of the pump characteristics hn = f(vol_flow) (s2/m5)";
  Real a2(start=0)
    "x coef. of the pump characteristics hn = f(vol_flow) (s/m2)";
  Real a3(start=43.15)
    "Constant coef. of the pump characteristics hn = f(vol_flow) (m)";
  Real b1(start=-3.7751)
    "x^2 coef. of the pump efficiency characteristics rh = f(vol_flow) (s2/m6)";
  Real b2(start=3.61)
    "x coef. of the pump efficiency characteristics rh = f(vol_flow) (s/m3)";
  Real b3(start=-0.0075464)
    "Constant coef. of the pump efficiency characteristics rh = f(vol_flow) (s.u.)";
  Real rhmin(start=0.20) "Minimum efficiency to avoid zero crossings";
protected
  constant Modelica.SIunits.Acceleration g=Modelica.Constants.g_n
    "Gravity constant";
public
  Real rh "Hydraulic efficiency";
  Modelica.SIunits.Height hn(start=10) "Pump head";
  Real R(start=1) "Reduced rotational speed";
  Modelica.SIunits.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.SIunits.VolumeFlowRate Qv(start=0.5) "Volume flow rate";
  Modelica.SIunits.VolumeFlowRate Qv0(start=0.5) "Volume flow rate";
  Modelica.SIunits.Power Wh "Hydraulic power";
  Modelica.SIunits.Power Wm "Mechanical power";
  Modelica.SIunits.SpecificEnthalpy deltaH
    "Specific enthalpy variation between the outlet and the inlet";
   MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP "Singular pressure loss";
  Modelica.Blocks.Interfaces.RealInput VRot annotation (Placement(
        transformation(extent={{0,-142},{40,-102}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
equation
  deltaH = h_out - h_in;
  deltaP = P_out - P_in;
  deltaP = rhom*g*hn;
  Q_in + Q_out = 0;
  Q = Q_in;
  Q = Qv*rhom;
  Q = max(Qv0*rhom,Qeps);
  /* Energy balance equation */
  if adiabatic_compression then
    deltaH = 0;
  else
    deltaH = g*hn/rh;
  end if;
  /* Reduced rotational speed */
  R = VRot/VRotn;
  /* Pump characteristics */
  hn = noEvent(a1*Qv0*abs(Qv0) + a2*Qv0*R + a3*R^2);
  rh = noEvent(max(if (abs(R) > eps) then b1*Qv*abs(Qv)/R^2 + b2*Qv/R + b3 else b3, rhmin));
  /* Mechanical power */
  Wm = Q*deltaH/rm;
  /* Hydraulic power */
  Wh = Qv*deltaP/rh;
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
    Window(
      x=0.03,
      y=0.02,
      width=0.95,
      height=0.95),
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
"), DymolaStoredErrors);
end StaticCentrifugalPump;
