within MetroscopeModelingLibrary.FlueGases.Machines;
model AirCompressor
   replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel(P_in(start=1e5), P_out(start=45e5),h_in(start=1e5), h_out(start=1.2e5), redeclare package
              Medium =
        FlueGasesMedium);
public
  Real tau(start=15);
  Real eta_is(start=0.8) "Nominal isentropic efficiency";
  Modelica.Units.SI.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.SpecificEnthalpy His(start=1e6);
  FlueGasesMedium.ThermodynamicState state_is;
  Modelica.Blocks.Interfaces.RealOutput Wmech annotation (Placement(
        transformation(extent={{100,70},{140,110}}), iconTransformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={114,86})));
equation
  Q_in + Q_out = 0;
  Q = Q_in;
  /* Compression ratio */
  tau = P_out/P_in;
  /* Fluid specific enthalpy after the expansion */
  h_out = (His - h_in + eta_is*h_in)/eta_is;
  /* Mechanical power produced by the turbine */
  Wmech = Q*(h_in - h_out);
  /*Chemical balance */
  Q_in*Xi_in =- Q_out*Xi_out;
  /* Isentropic  expansion */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in));
  His = Medium.specificEnthalpy(state_is);
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{100,60},{100,40},{100,-40},{100,-60},{80,-66},{-80,-100},{
              -100,-100},{-100,-80},{-100,77.5391},{-100,100},{-80,100},{80,68},
              {100,60}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,58},{92,40},{92,-40},{92,-54},{74,-60},{-72,-90},{-92,-94},
              {-92,-72},{-92,70},{-92,92},{-72,90},{72,62},{92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-60,86},{-60,-86}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-20,78},{-20,-78}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{20,72},{20,-70}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{60,62},{60,-60}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{-76,2},{74,-2}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end AirCompressor;
