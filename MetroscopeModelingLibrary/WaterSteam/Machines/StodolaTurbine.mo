within MetroscopeModelingLibrary.WaterSteam.Machines;
model StodolaTurbine
   replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
    extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel(P_in(start=60e5), P_out(start=55e5),h_in(start=2.7e6), h_out(start=2.6e6), redeclare
      package Medium =
        WaterSteamMedium);

  connector InputReal = input Real;
  connector InputPerUnit = input Modelica.Units.SI.PerUnit;

  InputReal Cst(start=1.e7) "Stodola's ellipse coefficient";
  Modelica.Units.SI.Area area_nz(start=1) "Nozzle area";
  InputPerUnit eta_nz(start=1.0)
    "Nozzle efficency (eta_nz < 1 - turbine with nozzle - eta_nz = 1 - turbine without nozzle)";
  InputPerUnit eta_is(start=0.8) "Nominal isentropic efficiency";
  Modelica.Units.SI.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.MassFraction x_in(start=1);
  Modelica.Units.SI.MassFraction x_out(start=0.9);
  Modelica.Units.SI.MassFraction x_inner(start=0.9);
  Modelica.Units.SI.MassFraction xm(start=0.9);
  Modelica.Units.SI.SpecificEnthalpy Hre(start=1e6);
  Modelica.Units.SI.SpecificEnthalpy His(start=1e6);
  Modelica.Units.SI.Velocity u_out(start=0);
  Medium.ThermodynamicState state_is;
  Modelica.Units.SI.Power Wmech;
  Electrical.Connectors.C_power C_power annotation (Placement(transformation(
          extent={{100,70},{128,100}}), iconTransformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={114,86})));


equation
  Q_in + Q_out = 0;
  Q = Q_in;
  /* Stodola's ellipse law */
  Q = sqrt((P_in^2 - P_out^2)/(Cst*T_in*x_in));
  /* Average vapor mass fraction during the expansion */
  xm = (x_in + x_inner)/2;
  /* Fluid specific enthalpy after the expansion */
  Hre - h_in = xm*eta_is*(His - h_in);
  /* Fluid specific enthalpy at the outlet of the nozzle */
  u_out = Q/rho_out/area_nz;
  h_out - Hre = (1 - eta_nz)*u_out^2/2;
  /* Mechanical power produced by the turbine */
  Wmech = C_power.W;
  Wmech = Q*(h_in - h_out);
  /* Vapor fractions */
  x_in = MetroscopeModelingLibrary.WaterSteam.Functions.VaporMassFraction(P_in,h_in);
  x_out = MetroscopeModelingLibrary.WaterSteam.Functions.VaporMassFraction(P_out,h_out);
  x_inner = MetroscopeModelingLibrary.WaterSteam.Functions.VaporMassFraction(P_out,Hre);
  /* Isentropic  expansion */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in));
  His = Medium.specificEnthalpy(state_is);
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Window(
      x=0.03,
      y=0.02,
      width=0.95,
      height=0.95),
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
          pattern=LinePattern.None)}),
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
end StodolaTurbine;
