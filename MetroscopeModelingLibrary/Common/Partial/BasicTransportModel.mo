within MetroscopeModelingLibrary.Common.Partial;
model BasicTransportModel
  replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  extends MetroscopeModelingLibrary.Common.Constants.Constants;
  Modelica.Units.SI.AbsolutePressure P_in(start=1e5) "Inlet Pressure";
  parameter Modelica.Units.SI.AbsolutePressure P_in_0 = 10e5 "Nominal Inlet Pressure";
  Modelica.Units.SI.AbsolutePressure P_out(start=0.9e5) "Outlet Pressure";
  Modelica.Units.SI.AbsolutePressure Pm(start=1.e5) "Average fluid pressure";
  parameter Modelica.Units.SI.MassFlowRate Q_in_0 = 100 "Inlet nominal Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_in(start=Q_in_0) "Inlet Mass flow rate";
  parameter Modelica.Units.SI.MassFlowRate Q_out_0 = Q_in_0 "Outlet nominal Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_out(start=Q_out_0) "Outlet Mass flow rate";
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
  Medium.MassFraction Xi_in[Medium.nXi];
  Medium.MassFraction Xi_out[Medium.nXi];
  Medium.ThermodynamicState state_in;
  Medium.ThermodynamicState state_out;

  Real W; // Heat Loss
  Real DM; // Mass Loss
  Real DP; // Pressure Loss
  Medium.MassFraction DXi[Medium.nXi]; // Species loss

  Connectors.FluidInlet C_in( redeclare package Medium = Medium)  annotation (Placement(transformation(extent={{-110,
            -10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (
      Placement(transformation(extent={{92,-10},{112,10}}), iconTransformation(
          extent={{92,-10},{112,10}})));
equation
  Q_in = C_in.Q;
  Q_out = C_out.Q;
  Qm=(Q_in-Q_out)/2;

  P_in = C_in.P;
  P_out = C_out.P;
  Pm = (P_in + P_out)/2;
  hm = (h_in+h_out)/2;


  h_in = inStream(C_in.h_outflow);
  Xi_in = inStream(C_in.Xi_outflow);
  h_out = C_out.h_outflow;
  Xi_out = C_out.Xi_outflow;

  // Never used with no flow reversal:
  C_in.Xi_outflow = zeros(Medium.nXi);
  C_in.h_outflow = 1e5;


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

  // Conservation equations
  Q_in + Q_out = DM;
  homotopy(Q_in*h_in + Q_out*h_out, Q_in_0*h_in + Q_out_0*h_out) = W;
  //Q_in*h_in + Q_out*h_out = W;
  P_out - P_in = DP;
  homotopy(Q_in*Xi_in + Q_out*Xi_out, Q_in_0*Xi_in + Q_out_0*Xi_out) = DXi;
  //Q_in*Xi_in + Q_out*Xi_out = DXi;


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
