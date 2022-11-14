within MetroscopeModelingLibrary.FlueGases.Machines;
model GasTurbine

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputReal tau(start=15, min = 1) "Compression rate";
  Real eta_is(start=0.8, min=0, max=1) "Nominal isentropic efficiency";
  Inputs.InputReal eta_mech(start=1, min=0, max=1) "Nominal mechanical efficiency";

  Units.SpecificEnthalpy h_is(start=1e6) "Isentropic compression outlet enthalpy";
  FlueGasesMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";

  Units.Power Wmech;

  Inputs.InputNotUsed dummy; // To keep local balance
  Modelica.Blocks.Interfaces.RealInput W_compressor "Declared as Input because the power is defined by the compressor" annotation (Placement(transformation(
        extent={{-21,-21},{21,21}},
        rotation=180,
        origin={-92,100}), iconTransformation(
        extent={{-15,-15},{15,15}},
        rotation=180,
        origin={-100,60})));
  Power.Connectors.Outlet C_W_out(dummy = dummy) annotation (Placement(transformation(extent={{90,90},{110,110}}), iconTransformation(extent={{90,90},{110,110}})));
equation

  /* Compression ratio */
  tau = P_in/P_out;

  /* Fluid specific enthalpy after the expansion */
  h_out-h_in = eta_is*(h_is-h_in);

  /* Mechanical power produced by the turbine */
  Wmech = - C_W_out.W;
  Wmech = eta_mech*Q*(h_in - h_out) - W_compressor;

  /* Isentropic  expansion */
  state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in),Xi);
  h_is = Medium.specificEnthalpy(state_is);

  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{-100,60},{-100,40},{-100,-40},{-100,-60},{-80,-66},{80,-100},{100,-100},{100,-80},{100,77.5391},{100,100},{80,100},{-80,68},{-100,60}},
          lineColor={95,95,95},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{-92,58},{-92,40},{-92,-40},{-92,-54},{-74,-60},{72,-90},{92,-94},{92,-72},{92,70},{92,92},{72,90},{-72,62},{-92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{66,86},{66,-86}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{22,78},{22,-78}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-16,68},{-16,-68}},
          color={95,95,95},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-60,58},{-60,-60}},
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
end GasTurbine;
