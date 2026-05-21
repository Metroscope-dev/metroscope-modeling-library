within MetroscopeModelingLibrary.MultiFluid.Machines;
model AirCompressor

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputReal tau(start=15, min = 1) "Compression rate";
  Inputs.InputReal eta_is(start=0.8, min=0, max=1) "Nominal isentropic efficiency";
  Real Q_reduced "Compressor reduced mass flow";
  Inputs.InputReal tau_moist(start=2, min = 1) "Compression rate of the moist air section";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage eta_is_decrease(min = 0, max=100) "percentage decrease of eta_is";
  Units.Percentage tau_decrease(min = 0, max=100) "percentage decrease of tau";


  Power.Connectors.Inlet C_W_in annotation (Placement(transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{90,50},{110,70}})));
  RefMoistAir.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  RefMoistAir.Machines.AirCompressor MoistAirCompressor annotation (Placement(transformation(extent={{-60,-8},{-40,8}})));
  Converters.RefMoistAir_to_FlueGases refMoistAir_to_FlueGases annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  FlueGases.Machines.AirCompressor FlueGasesCompressor annotation (Placement(transformation(extent={{40,-8},{60,8}})));
  FlueGases.Connectors.Outlet outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
equation

  // Failure modes
  if not faulty then
    eta_is_decrease = 0;
    tau_decrease = 0;
  end if;

  /* Compression ratio */
  tau*(1-tau_decrease/100) = FlueGasesCompressor.P_out/MoistAirCompressor.P_in;
  tau_moist = MoistAirCompressor.tau;

  /* Isentropic efficiency */
  FlueGasesCompressor.eta_is = eta_is*(1-eta_is_decrease/100);
  MoistAirCompressor.eta_is = eta_is*(1-eta_is_decrease/100);

  /* Output variable */
  Q_reduced =  MoistAirCompressor.Q_reduced;


  connect(inlet, MoistAirCompressor.C_in) annotation (Line(points={{-100,0},{-60,0}}, color={0,255,128}));
  connect(MoistAirCompressor.C_out, refMoistAir_to_FlueGases.inlet) annotation (Line(points={{-40,0},{-10,0}}, color={0,255,128}));
  connect(refMoistAir_to_FlueGases.outlet, FlueGasesCompressor.C_in) annotation (Line(points={{10,0},{40,0}}, color={95,95,95}));
  connect(FlueGasesCompressor.C_out, outlet) annotation (Line(points={{60,0},{100,0}}, color={95,95,95}));
  connect(MoistAirCompressor.C_W_in, C_W_in) annotation (Line(
      points={{-40,6},{-40,60},{100,60}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(FlueGasesCompressor.C_W_in, C_W_in) annotation (Line(
      points={{60,6},{60,60},{100,60}},
      color={244,125,35},
      smooth=Smooth.Bezier));
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
          lineColor={0,0,0},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,20},{92,14},{92,-14},{92,-20},{76,-26},{-72,-50},{-92,-54},{-92,-40},{-92,40},{-92,54},{-70,50},{76,24},{92,20}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={170,255,213},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,255,128}),
        Line(
          points={{-66,38},{-66,-38}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{92,20},{92,14},{92,-14},{92,-20},{76,-26},{0,-38},{0,-38},{0,-38},{0,38},{0,38},{0,38},{76,24},{92,20}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={175,175,175}),
        Line(
          points={{6,26},{6,-26}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{24,22},{24,-22}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{56,19},{56,-19}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-30,32},{-30,-32}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Polygon(
          points={{-71,-2.5},{-65,-2.5},{-45,-2.5},{0,-7.5},{62,-7.5},{72,-7.5},{72,0.5},{72,0.5},{72,6.5},{63,7.5},{0,6.5},{-45,1.5},{-65,1.5},{-71,1.5},{-71,-0.5},{-71,-0.5},{-71,-2.5}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,20.5},{40,-20.5}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-12,29.25},{-12,-29.25}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-48,35.5},{-48,-35.5}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.Bezier)}));
end AirCompressor;
