within MetroscopeModelingLibrary.FlueGases.Machines;
model GasTurbine

  extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium,
    Q_0 = 500, rho_0 = 1) annotation (IconMap(primitivesVisible=false));

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  parameter Units.Yield eta_is_constant = 0.8;
  Inputs.InputReal tau(start=15, min = 1) "Compression rate";
  parameter Units.Yield eta_mech = 0.99 "Nominal mechanical efficiency";

  Units.SpecificEnthalpy h_is(start=1e6) "Isentropic compression outlet enthalpy";
  FlueGasesMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";

  Units.Power W_shaft;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage eta_is_decrease(min = 0, max=100) "percentage decrease of eta_is";

  Power.Connectors.Outlet C_W_shaft annotation (Placement(transformation(extent={{90,90},{110,110}}), iconTransformation(extent={{90,90},{110,110}})));
  Utilities.Interfaces.GenericReal eta_is(start=eta_is_constant) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-100,80}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-100,80})));
equation

  // Failure modes
  if not faulty then
    eta_is_decrease = 0;
  end if;

  /* Compression ratio */
  tau = P_in/P_out;

  /* Fluid specific enthalpy after the expansion */
  DH = eta_is*(1-eta_is_decrease/100)*(h_is - h_in);

  /* Mechanical power produced by the turbine */
  W_shaft = - C_W_shaft.W;
  W_shaft = - eta_mech * W;

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
