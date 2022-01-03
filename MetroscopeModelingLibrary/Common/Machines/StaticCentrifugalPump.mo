within MetroscopeModelingLibrary.Common.Machines;
model StaticCentrifugalPump "Static centrifugal pump"
  extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;
  parameter Boolean adiabatic_compression=false
    "true: compression at constant enthalpy - false: compression with varying enthalpy";

  connector InputReal = input Real;
  connector InputMassFlowRate = input Modelica.Units.SI.MassFlowRate;

  MetroscopeModelingLibrary.Common.Units.AngularVelocity_rpm VRotn(start=1400)
    "Nominal rotational speed";
  InputReal rm(start=0.85)
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
  Real rh "Hydraulic efficiency";
  Modelica.Units.SI.Height hn(start=10) "Pump head";
  Real R(start=1) "Reduced rotational speed";
  InputMassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv(start=0.5) "Volume flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv0(start=0.5) "Volume flow rate";
  Modelica.Units.SI.Power Wh "Hydraulic power";
  Modelica.Units.SI.Power Wm "Mechanical power";
  Modelica.Units.SI.SpecificEnthalpy deltaH
    "Specific enthalpy variation between the outlet and the inlet";
   MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP "Singular pressure loss";
  MetroscopeModelingLibrary.Common.Connectors.RealOutput VRot annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-120}),                            iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,-120})));
  Electrical.Connectors.C_power C_power annotation (Placement(transformation(extent={{
            -18,118},{22,158}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,122})));
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

  C_power.W = Wm;
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
