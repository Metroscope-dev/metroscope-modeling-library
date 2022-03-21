within MetroscopeModelingLibrary.Partial;
package PartialTransport
  partial model PartialTransportModel "Basic fluid transport brick for all components"
    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Units;
    import MetroscopeModelingLibrary.Partial.PartialTransport;
    // Initialization parameters

    // ------ Input Quantities ------
    // Enthalpies
    extends PartialTransport.PartialTransportH;
    // Mass flow rate
    extends PartialTransport.PartialTransportQ;
    // Pressures
    extends PartialTransport.PartialTransportP;
    // Mass fractions
    extends PartialTransport.PartialTransportXi;


    // ------ Computed Quantities ------
    // Densities
    Units.Density rho_in(start=998) "Inlet Fluid density";
    Units.Density rho_out(start=998) "Outlet Fluid density";
    Units.Density rhom(start=998) "Average Fluid density";

    // Volumic flow rates
    Units.VolumeFlowRate Qv_in(start=0.1) "inlet volume flow rate";
    Units.VolumeFlowRate Qv_out(start=0.1) "outlet volume flow rate";
    Units.VolumeFlowRate Qvm(start=0.1) "Mean volume flow rate";

    // Temperatures
    Units.Temperature T_in(start=290) "Fluid temperature";
    Units.Temperature T_out(start=291) "Fluid temperature";
    Units.Temperature Tm(start=290) "Fluid temperature";

    // ------ States ------
    Medium.ThermodynamicState state_in;
    Medium.ThermodynamicState state_out;

    // ------ Conservation variables ------
    Units.MassFlowRate DM(nominal=0, start=0); // Mass Loss
    Units.DifferentialPressure DP(nominal=0, start=0); // Pressure Loss
    Units.Power W(nominal=0, start=0); // Heat Loss
    Units.MassFraction DXi[Medium.nXi] "species mass fraction variation in component";

    // Connectors
  equation
    // ------ States ------
    state_in = Medium.setState_phX(P_in, h_in, Xi_in);
    state_out = Medium.setState_phX(P_out, h_out, Xi_out);

    // ------ Computed Quantities ------
    T_in = Medium.temperature(state_in);
    T_out = Medium.temperature(state_out);
    Tm = (T_in + T_out)/2;

    rho_in = Medium.density(state_in);
    rho_out = Medium.density(state_out);
    rhom = (rho_in + rho_out)/2;

    Qv_in = Q_in / rho_in;
    Qv_out = Q_out / rho_out;
    Qvm = Qm / rhom;

    // ------ Conservation equations ------
    Q_in + Q_out = DM;
    P_out - P_in = DP;
    Q_in*h_in + Q_out*h_out = W;
    Q_in*Xi_in + Q_out*Xi_out = DXi;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,46},{100,-48}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false)));
  end PartialTransportModel;

  partial model PartialTransportH
    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Units;
    Units.SpecificEnthalpy h_in(start=100000) "Inlet specific enthalpy";
    Units.SpecificEnthalpy h_out(start=100000) "Outlet specific enthalpy";
    Units.SpecificEnthalpy hm(start=100000) "Average specific enthalpy";

    // Connectors
    Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    h_out = C_out.h_outflow;
    h_in = inStream(C_in.h_outflow);
    C_in.h_outflow = 1e5; // Never used, as it is assumed that there is no flow reversal

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,46},{100,-48}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,60},{100,-60}},
            lineColor={28,108,200},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end PartialTransportH;

  partial model PartialTransportQ
    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Units;

    Units.MassFlowRate Q_in(start=100) "Inlet Mass flow rate";
    Units.MassFlowRate Q_out(start=100) "Outlet Mass flow rate";
    Units.MassFlowRate Qm(start=100) "Mean Mass flow rate";

    // Connectors
    Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    Q_in = C_in.Q;
    Q_out = C_out.Q;
    Qm = (Q_in-Q_out)/2;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,46},{100,-48}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,60},{100,-60}},
            lineColor={28,108,200},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end PartialTransportQ;

  partial model PartialTransportP
    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Units;

    Units.Pressure P_in(start=1e5) "Inlet Pressure";
    Units.Pressure P_out(start=0.9e5) "Outlet Pressure";
    Units.Pressure Pm(start=1.e5) "Average fluid pressure";
    // Connectors
    Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    P_in = C_in.P;
    P_out = C_out.P;
    Pm = (P_in + P_out)/2;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,46},{100,-48}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,60},{100,-60}},
            lineColor={28,108,200},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end PartialTransportP;

  partial model PartialTransportXi
    replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
    import MetroscopeModelingLibrary.Units;

    Units.MassFraction Xi_in[Medium.nXi] "Inlet species mass fraction";
    Units.MassFraction Xi_out[Medium.nXi] "Outlet species mass fraction";
    // Connectors
    Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  equation
    Xi_in = inStream(C_in.Xi_outflow);
    Xi_out = C_out.Xi_outflow;
    C_in.Xi_outflow = zeros(Medium.nXi);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,46},{100,-48}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
            extent={{-100,60},{100,-60}},
            lineColor={28,108,200},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end PartialTransportXi;
end PartialTransport;
