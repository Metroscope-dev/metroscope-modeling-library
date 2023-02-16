within MetroscopeModelingLibrary.WaterSteam.Machines;
model StodolaTurbine
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    Q_0=1500,
    P_in_0=60e5,
    P_out_0=55e5,
    h_in_0 = 2.7e6,
    h_out_0= 2.6e6,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputCst Cst "Stodola's ellipse coefficient";
  Inputs.InputYield eta_is(start=0.8) "Nominal isentropic efficiency";
  Inputs.InputYield eta_nz(start=1.0) "Nozzle efficency (eta_nz < 1, turbine with nozzle ; eta_nz = 1, turbine without nozzle)";
  Utilities.Units.Area area_nz(start=1) "Nozzle area";
  Utilities.Units.Velocity u_out(start=100);

  Utilities.Units.MassFraction x_in(start=x_in_0);
  Utilities.Units.MassFraction x_inner(start=x_inner_0);
  Utilities.Units.MassFraction xm(start=xm_0);

  Utilities.Units.SpecificEnthalpy h_real(start=h_out_0);
                                                // Enthalpy after real decompression
  Utilities.Units.SpecificEnthalpy h_is(start=h_out_0/0.8);
                                                  // Enthalpy after isentropic decompression
  Medium.ThermodynamicState state_is; // Thermodynamic state after isentropic decompression

  // Liq/Vap enthalpies
  Utilities.Units.SpecificEnthalpy h_vap_in(start=h_vap_in_0);
  Utilities.Units.SpecificEnthalpy h_vap_out(start=h_vap_out_0);
  Utilities.Units.SpecificEnthalpy h_liq_in(start=h_liq_in_0);
  Utilities.Units.SpecificEnthalpy h_liq_out(start=h_liq_out_0);

  // Initialization parameters
  parameter Utilities.Units.MassFraction x_inner_0=min((h_out_0 - h_liq_out_0)/(h_vap_out_0 - h_liq_out_0), 1);
  parameter Utilities.Units.MassFraction xm_0=(x_inner_0 + x_in_0)/2;
  parameter Utilities.Units.MassFraction x_in_0=min((h_in_0 - h_liq_in_0)/(h_vap_in_0 - h_liq_in_0), 1);

  Power.Connectors.Outlet C_W_out annotation (Placement(transformation(extent={{90,74},{110,94}}), iconTransformation(extent={{90,74},{110,94}})));
protected
  parameter Utilities.Units.SpecificEnthalpy h_vap_in_0=WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_in_0));
  parameter Utilities.Units.SpecificEnthalpy h_liq_in_0=WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_in_0));
  parameter Utilities.Units.SpecificEnthalpy h_vap_out_0=WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_out_0));
  parameter Utilities.Units.SpecificEnthalpy h_liq_out_0=WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_out_0));

equation
  // Stodola's ellipse law
  if use_homotopy then
    Q = homotopy(sqrt((P_in^2 - P_out^2)/(Cst*T_in*x_in)), sqrt((P_in*P_in_0 - P_out*P_out_0)/(Cst*T_in_0*x_in_0)));
  else
    Q = sqrt((P_in^2 - P_out^2)/(Cst*T_in*x_in));
 end if;

  // Isentropic expansion
  state_is = Medium.setState_psX(P_out, Medium.specificEntropy(state_in));
  h_is = Medium.specificEnthalpy(state_is);

  // Fluid specific enthalpy after the expansion
  h_real - h_in = xm*eta_is*(h_is - h_in);

  // Nozzle outlet
  u_out = Q/(rho_out*area_nz);
  h_out - h_real = (1 - eta_nz)*u_out^2/2;

  // Mechanical power produced by the turbine
  W = C_W_out.W;

  // Vapor fractions
  h_vap_in = Medium.dewEnthalpy(Medium.setSat_p(P_in));
  h_liq_in = Medium.bubbleEnthalpy(Medium.setSat_p(P_in));
  x_in = min((h_in - h_liq_in)/(h_vap_in - h_liq_in), 1);

  h_vap_out = Medium.dewEnthalpy(Medium.setSat_p(P_out));
  h_liq_out = Medium.bubbleEnthalpy(Medium.setSat_p(P_out));
  x_inner = min((h_real - h_liq_out)/(h_vap_out - h_liq_out), 1);

  xm = (x_in + x_inner)/2;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{-100,60},{-100,40},{-100,-40},{-100,-60},{-80,-66},{80,-100},
              {100,-100},{100,-80},{100,77.5391},{100,100},{80,100},{-80,68},{
              -100,60}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{-92,58},{-92,40},{-92,-40},{-92,-54},{-74,-60},{72,-90},{92,
              -94},{92,-72},{92,70},{92,92},{72,90},{-72,62},{-92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{66,86},{66,-86}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{22,78},{22,-78}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-20,68},{-20,-68}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-60,60},{-60,-58}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{74,2},{-76,-2}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end StodolaTurbine;
