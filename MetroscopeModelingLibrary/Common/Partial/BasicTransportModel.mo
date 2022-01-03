within MetroscopeModelingLibrary.Common.Partial;
model BasicTransportModel
  replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  extends MetroscopeModelingLibrary.Common.Constants.Constants;
  Modelica.Units.SI.AbsolutePressure P_in(start=1e5) "Inlet Pressure";
  Modelica.Units.SI.AbsolutePressure P_out(start=0.9e5) "Outlet Pressure";
  Modelica.Units.SI.AbsolutePressure Pm(start=1.e5) "Average fluid pressure";
  Modelica.Units.SI.MassFlowRate Q_in(start=100) "Inlet Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_out(start=100) "Outlet Mass flow rate";
  Modelica.Units.SI.MassFlowRate Qm(start=100) "Mean Mass flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv_in(start=0.1) "inlet volume flow rate";
  Modelica.Units.SI.VolumeFlowRate Qv_out(start=0.1) "outlet volume flow rate";
  Modelica.Units.SI.VolumeFlowRate Qvm(start=0.1) "mean volume flow rate";
  Modelica.Units.SI.SpecificEnthalpy h_in(start=100000)
    "Inlet specific enthalpy";
  Modelica.Units.SI.SpecificEnthalpy h_out(start=100000)
    "Outlet specific enthalpy";
  Modelica.Units.SI.SpecificEnthalpy hm(start=100000)
    "Average specific enthalpy";
  Modelica.Units.SI.Density rho_in(start=998) "Fluid density";
  Modelica.Units.SI.Density rho_out(start=998) "Fluid density";
  Modelica.Units.SI.Density rhom(start=998) "Fluid density";
  Modelica.Units.SI.Temperature T_in(start=290) "Fluid temperature";
  Modelica.Units.SI.Temperature T_out(start=291) "Fluid temperature";
  Modelica.Units.SI.Temperature Tm(start=290) "Fluid temperature";
  Modelica.Units.SI.MassFlowRate Qi_in[Medium.nXi];
  Medium.MassFraction Xi_in[Medium.nXi];
  Modelica.Units.SI.MassFlowRate Qi_out[Medium.nXi];
  Medium.MassFraction Xi_out[Medium.nXi];
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;
  Connectors.FluidInlet C_in( redeclare package Medium = Medium)  annotation (Placement(transformation(extent={{-110,
            -10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (
      Placement(transformation(extent={{92,-10},{112,10}}), iconTransformation(
          extent={{92,-10},{112,10}})));
equation
  Q_in = C_in.Q;
  Q_out = C_out.Q;
  Qm=(Q_in-Q_out)/2;
  Qi_in = C_in.Qi;
  Qi_out = C_out.Qi;
  P_in = C_in.P;
  P_out = C_out.P;
  Pm = (P_in + P_out)/2;
  hm = (h_in+h_out)/2;

  if Q_in > 0 then
    C_in.H = Q_in*  C_in.h_vol;
  else
    C_out.H = Q_out*  C_out.h_vol;
  end if;
  C_in.H = h_in *  Q_in;
  C_out.H = h_out * Q_out;
  if Q_in > 0 then
    C_in.Qi = Q_in*  C_in.Xi_vol;
  else
    C_out.Qi = Q_out*  C_out.Xi_vol;
  end if;
  C_in.Qi = Xi_in *  Q_in;
  C_out.Qi = Xi_out * Q_out;
  /* Fluid thermodynamic properties */
  state_in = Medium.setState_phX(P_in, h_in,Xi_in);
  state_out = Medium.setState_phX(P_out, h_out,Xi_out);
  T_in = Medium.temperature(state_in);
  rho_in = Medium.density(state_in);
  T_out = Medium.temperature(state_out);
  rho_out = Medium.density(state_out);
  Tm = (T_in+T_out)/2;
  rhom = (rho_in+rho_out)/2;

  Qv_in=Q_in/rho_in;
  Qv_out=Q_out/rho_out;
  Qvm=Qm/rhom;

     annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,40},{80,-40}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{-92,0},{-94,0}}, color={0,0,0}),
        Line(points={{80,0},{96,0}}, color={0,0,0})}));
end BasicTransportModel;
