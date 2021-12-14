within MetroscopeModelingLibrary.MoistAir.Machines;
model AirCompressor
   package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
    extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel(P_in(start=1e5), P_out(start=45e5),h_in(start=1e5), h_out(start=1.2e5), redeclare
      package Medium =
        MoistAirMedium);
public
  Real tau(start=15);
  Real eta_is(start=0.8) "Nominal isentropic efficiency";
  Modelica.SIunits.MassFlowRate Q(start=500) "Mass flow rate";
  Modelica.SIunits.SpecificEnthalpy His(start=1e6);
  MoistAirMedium.ThermodynamicState state_is;
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
    Window(
      x=0.03,
      y=0.02,
      width=0.95,
      height=0.95),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{100,40},{100,-40},{-100,-100},{-100,100},{100,40}},
          lineColor={0,0,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li></li>
</ul>
</html>
"), DymolaStoredErrors);
end AirCompressor;
