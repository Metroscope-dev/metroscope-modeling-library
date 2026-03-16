within MetroscopeModelingLibrary.FlueGases.Machines;
model AirCompressor

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium,
    Q_0 = 500, rho_0 = 1) annotation (IconMap(primitivesVisible=false));

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Units.SpecificEnthalpy h_is(start=1e6) "Isentropic compression outlet enthalpy";
  FlueGasesMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";
  Real Q_reduced "Compressor reduced mass flow";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage eta_is_decrease(min = 0, max=100) "percentage decrease of eta_is";
  Units.Percentage tau_decrease(min = 0, max=100) "percentage decrease of tau";

  Power.Connectors.Inlet C_W_in annotation (Placement(transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{90,50},{110,70}})));
  Utilities.Interfaces.GenericReal tau annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-58}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-60})));
  Utilities.Interfaces.GenericReal eta_is annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,-48}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,-50})));
equation

  // Failure modes
  if not faulty then
    eta_is_decrease = 0;
    tau_decrease = 0;
  end if;

  /* Compression ratio */
  tau*(1-tau_decrease/100) = P_out/P_in;

  /* Fluid specific enthalpy after the expansion */
  DH*eta_is*(1-eta_is_decrease/100) = h_is - h_in;

  /* Mechanical power from the turbine */
  C_W_in.W = W;

  /* Isentropic compression */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in),Xi);
  h_is = Medium.specificEnthalpy(state_is);

  /* Output variable */
  Q_reduced = Q * sqrt(T_in) / P_in;

  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-80},{100,80}},
        initialScale=0.4)),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-80},{100,80}},
        initialScale=0.4),
                     graphics={Polygon(
          points={{100,26},{100,14},{100,-14},{100,-26},{80,-32},{-80,-60},{-100,-64},{-100,-40},{-100,40},{-100,64},{-80,60},{80,30},{100,26}},
          lineColor={95,95,95},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,20},{92,14},{92,-14},{92,-20},{76,-26},{-72,-50},{-92,-54},{-92,-40},{-92,40},{-92,54},{-70,50},{76,24},{92,20}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-66,38},{-66,-38}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{6,26},{6,-26}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{24,22},{24,-22}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{56,19},{56,-19}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-30,32},{-30,-32}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Polygon(
          points={{-71,-2.5},{-65,-2.5},{-45,-2.5},{0,-7.5},{62,-7.5},{72,-7.5},{72,0.5},{72,0.5},{72,6.5},{63,7.5},{0,6.5},{-45,1.5},{-65,1.5},{-71,1.5},{-71,-0.5},{-71,-0.5},{-71,-2.5}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,20.5},{40,-20.5}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-12,29.25},{-12,-29.25}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-48,35.5},{-48,-35.5}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier)}));
end AirCompressor;
