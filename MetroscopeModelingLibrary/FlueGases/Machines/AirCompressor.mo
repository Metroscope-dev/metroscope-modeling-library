within MetroscopeModelingLibrary.FlueGases.Machines;
model AirCompressor

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputReal tau(start=15, min = 1) "Compression rate";
  Inputs.InputReal eta_is(start=0.8, min=0, max=1) "Nominal isentropic efficiency";

  Units.SpecificEnthalpy His(start=1e6) "Isentropic compression outlet enthalpy";
  FlueGasesMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";

  Units.Power Wmech;


  Power.Connectors.Inlet C_W_in annotation (Placement(transformation(extent={{90,90},{110,110}}), iconTransformation(extent={{90,90},{110,110}})));
equation

  /* Compression ratio */
  tau = P_out/P_in;

  /* Fluid specific enthalpy after the expansion */
  (h_out-h_in)*eta_is = His - h_in;

  /* Mechanical power from the turbine */
  Wmech = - Q*(h_in - h_out);
  C_W_in.W =  Wmech;

  /* Isentropic compression */
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
          points={{100,60},{100,40},{100,-40},{100,-60},{80,-66},{-80,-100},{-100,-100},{-100,-80},{-100,77.5391},{-100,100},{-80,100},{80,68},{100,60}},
          lineColor={95,95,95},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,58},{92,40},{92,-40},{92,-54},{74,-60},{-72,-90},{-92,-94},{-92,-72},{-92,70},{-92,92},{-72,90},{72,62},{92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-62,86},{-62,-86}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-22,78},{-22,-78}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{18,68},{18,-68}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{56,60},{56,-58}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{74,2},{-76,-2}},
          lineThickness=0.5,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={95,95,95})}));
end AirCompressor;