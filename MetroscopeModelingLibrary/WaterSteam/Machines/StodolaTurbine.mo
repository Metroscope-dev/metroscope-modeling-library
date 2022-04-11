within MetroscopeModelingLibrary.WaterSteam.Machines;
model StodolaTurbine
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    P_in_0=60e5,
    P_out_0=55e5,
    h_in(start=2.7e6),
    h_out(start=2.6e6),
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputCst Cst "Stodola's ellipse coefficient";
  Inputs.InputYield eta_is(start=0.8) "Nominal isentropic efficiency";
  Inputs.InputYield eta_nz(start=1.0) "Nozzle efficency (eta_nz < 1, turbine with nozzle ; eta_nz = 1, turbine without nozzle)";
  Units.Area area_nz(start=1) "Nozzle area";

  Units.Velocity u_out(start=0);

  Units.MassFraction x_in(start=1); // ok
  Units.MassFraction x_inner(start=0.9);
  Units.MassFraction xm(start=0.95);

  Units.SpecificEnthalpy Hre(start=1e6);
  Units.SpecificEnthalpy His(start=1e6); // ok

  // Liq/Vap enthalpies
  Units.SpecificEnthalpy h_vap_in(start=1e6);
  Units.SpecificEnthalpy h_vap_out(start=1e6);
  Units.SpecificEnthalpy h_liq_in(start=1e6);
  Units.SpecificEnthalpy h_liq_out(start=1e6);

  Medium.ThermodynamicState state_is; // ok
  //Units.OutletPower Wmech;

  Power.Connectors.PowerOutlet C_W_out annotation (Placement(transformation(extent={{90,74},{110,94}}), iconTransformation(extent={{90,74},{110,94}})));
equation

///*
  // Stodola's ellipse law
  Q = sqrt((P_in^2 - P_out^2)/(Cst*T_in*x_in));

  // Isentropic expansion
  state_is = Medium.setState_psX(P_out, Medium.specificEntropy(state_in)); // state_is
  His = Medium.specificEnthalpy(state_is);

  // Fluid specific enthalpy after the expansion
  Hre - h_in = xm*eta_is*(His - h_in);

  // Nozzle outlet
  u_out = Q/(rho_out*area_nz);
  h_out - Hre = (1 - eta_nz)*u_out^2/2;

  // Mechanical power produced by the turbine
  W = C_W_out.W;
  //Wmech = C_W_out.W;
  //Wmech = - W; //Q*(h_in - h_out);

  // Vapor fractions
  h_vap_in = Medium.dewEnthalpy(Medium.setSat_p(P_in));
  h_liq_in = Medium.bubbleEnthalpy(Medium.setSat_p(P_in));
  x_in = min((h_in - h_liq_in)/(h_vap_in - h_liq_in), 1);// MetroscopeModelingLibrary.WaterSteam.Functions.VaporMassFraction(P_in,h_in);

  h_vap_out = Medium.dewEnthalpy(Medium.setSat_p(P_out));
  h_liq_out = Medium.bubbleEnthalpy(Medium.setSat_p(P_out));
  x_inner = min((Hre - h_liq_out)/(h_vap_out - h_liq_out), 1);//MetroscopeModelingLibrary.WaterSteam.Functions.VaporMassFraction(P_out,Hre);

  xm = (x_in + x_inner)/2;
//*/
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
