within MetroscopeModelingLibrary.Common.Machines;
model StodolaTurbine "Stodola Turbine"
  extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;
public
  Real Cst(start=1.e7) "Stodola's ellipse coefficient";
  Modelica.Units.SI.Area area_nz(start=1) "Nozzle area";
  Real eta_nz(start=1.0)
    "Nozzle efficency (eta_nz < 1 - turbine with nozzle - eta_nz = 1 - turbine without nozzle)";
  Real eta_is(start=0.8) "Nominal isentropic efficiency";
  Modelica.Units.SI.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Blocks.Interfaces.RealInput Wmech annotation (Placement(
        transformation(extent={{0,-142},{40,-102}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-90})));
equation
  Q_in + Q_out = 0;
  Q = Q_in;
  /* Stodola's ellipse law */
  Q = sqrt((P_in^2 - P_out^2)/(Cst*Te*x_in));
  /* Average vapor mass fraction during the expansion */
  xm = (x_in + x_out)/2;
  /* Fluid specific enthalpy after the expansion */
  Hre - h_in = xm*eta_is*(His - h_in);
  /* Fluid specific enthalpy at the outlet of the nozzle */
  u_out = Q/rho_out/area_nz;
  h_out - Hre = (1 - eta_nz)*u_out^2/2;
  /* Mechanical power produced by the turbine */
  Wmech = Q*(h_in - h_out);
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
          points={{62,84},{62,-84}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{22,74},{22,-74}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-20,64},{-20,-64}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-60,54},{-60,-50}},
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
<p><b>Copyright &copy;Metroscope</b> </p>
<p>Metroscope Modeling Library</p>
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
