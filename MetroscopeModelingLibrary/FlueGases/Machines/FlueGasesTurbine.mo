within MetroscopeModelingLibrary.FlueGases.Machines;
model FlueGasesTurbine
   replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel(P_in(start=1e5), P_out(start=45e5),h_in(start=1e5), h_out(start=1.2e5), redeclare package
              Medium =
        FlueGasesMedium);

  connector InputPerUnit = input Modelica.Units.SI.PerUnit;

  InputPerUnit eta_mech(start=1) "";
  Modelica.Units.SI.PerUnit tau(start=15);
  InputPerUnit eta_is(start=0.8) "Nominal isentropic efficiency";
  Modelica.Units.SI.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.Units.SI.SpecificEnthalpy His(start=1e6);
  FlueGasesMedium.ThermodynamicState state_is;
  Modelica.Units.SI.Power Wmech;
  Electrical.Connectors.C_power         C_power annotation (Placement(
        transformation(extent={{100,70},{140,110}}), iconTransformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={114,86})));
  Modelica.Blocks.Interfaces.RealInput Wmech_compressor annotation (Placement(
        transformation(extent={{100,70},{140,110}}), iconTransformation(
        extent={{14,-14},{-14,14}},
        rotation=0,
        origin={-114,86})));
equation
  Q_in + Q_out = 0;
  Q = Q_in;
  /* Compression ratio */
  tau = P_in/P_out;
  /* Fluid specific enthalpy after the expansion */
  h_out-h_in = eta_is*(His-h_in);
  /* Mechanical power produced by the turbine */
  Wmech = C_power.W;
  Wmech = eta_mech*Q*(h_in - h_out) + Wmech_compressor;
  /*Chemical balance */
  Q_in*Xi_in =- Q_out*Xi_out;
  /* Isentropic  expansion */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in),Xi_out);
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
    Documentation(info="",
   revisions="<html>


<table>
<tr>
<th> Name</th> <th> Type </th> 
</tr>

<tr>
<th> Test</th> <th> Output </th> 
</tr>


</table>

<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end FlueGasesTurbine;
