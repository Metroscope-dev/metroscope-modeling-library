within MetroscopeModelingLibrary;
package RefMoistAir

  package Connectors
    extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;

    connector Inlet
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.Connectors.FluidInlet(redeclare package Medium =
            RefMoistAirMedium)                                                            annotation(IconMap(primitivesVisible=false));
      annotation (Icon(graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,255,128},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid)}));
    end Inlet;

    connector Outlet
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.Connectors.FluidOutlet(redeclare package Medium =
            RefMoistAirMedium)                                                             annotation(IconMap(primitivesVisible=false));
      annotation (Icon(graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,255,128},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end Outlet;
    annotation (Icon(graphics={
          Rectangle(
            extent={{20,30},{78,-28}},
            lineColor={0,255,128},
            lineThickness=1,
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-28,0},{20,0}},
            color={0,255,128},
            thickness=1),
          Rectangle(
            extent={{-78,26},{-28,-24}},
            lineColor={0,255,128},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Connectors;

  package BoundaryConditions
    model Source
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
                                                                                                                           redeclare package Medium =
            RefMoistAirMedium)                                                                                                                                           annotation (IconMap(primitivesVisible=false));

      parameter Real relative_humidity_0(min=0, max=1) = 0.1;
      Real relative_humidity(start=relative_humidity_0, min=0, max=1);
      Real pds;
      parameter Real k_mair = RefMoistAirMedium.k_mair;

    equation
      pds = RefMoistAirMedium.Utilities.pds_pT(P_out, T_out);
      Xi_out = {relative_humidity*k_mair/(P_out/pds - relative_humidity)};



      annotation (Icon(graphics={
            Ellipse(
              extent={{-80,60},{40,-60}},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid,
              lineThickness=0.5,
              pattern=LinePattern.None,
              lineColor={0,0,0})}));
    end Source;

    model Sink
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
                                                                                                                       redeclare package Medium =
            RefMoistAirMedium)                                                                                                                                       annotation (IconMap(primitivesVisible=
              false));

      parameter Real relative_humidity_0(min=0, max=1) = 0.1;
      Real relative_humidity(start=relative_humidity_0, min=0, max=1);
      Real pds;
      parameter Real k_mair = RefMoistAirMedium.k_mair;
    equation
      pds = RefMoistAirMedium.Utilities.pds_pT(P_in, T_in);
      Xi_in = {relative_humidity*k_mair/(P_in/pds - relative_humidity)};

      annotation (Icon(graphics={
            Ellipse(
              extent={{-40,60},{80,-60}},
              lineColor={0,255,128},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-30,50},{70,-50}},
              lineColor={0,255,128},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-18,38},{55,-35}}, color={0,255,128},
              thickness=1),
            Line(points={{-18,-38},{55,35}}, color={0,255,128},
              thickness=1)}));
    end Sink;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            extent={{-76,58},{44,-62}},
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid,
            lineThickness=1,
            pattern=LinePattern.None,
            lineColor={0,255,128}),
        Line(
          points={{54,0},{84,0}},
          color={0,255,128},
          thickness=1),
          Rectangle(
            extent={{42,12},{66,-12}},
            lineColor={0,255,128},
            lineThickness=1,
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid)}));
  end BoundaryConditions;

  package BaseClasses
    model FlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BaseClasses.FlowModel(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      Inputs.InputPower W_input(start=0);
      Inputs.InputDifferentialPressure DP_input(start=0);
    equation
      W = W_input;
      DP = DP_input;
    end FlowModel;

    model IsoPFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BaseClasses.IsoPFlowModel(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      Inputs.InputPower W_input(start=0);
    equation
      W = W_input;
    end IsoPFlowModel;

    model IsoHFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BaseClasses.IsoHFlowModel(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      Inputs.InputDifferentialPressure DP_input(start=0);
    equation
      DP = DP_input;
    end IsoHFlowModel;

    model IsoPHFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.BaseClasses.IsoPHFlowModel(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));
    end IsoPHFlowModel;
  annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
       Rectangle(
            extent={{-46,49},{46,-47}},
            lineColor={0,255,128},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1),
          Rectangle(
            extent={{26,18},{60,-16}},
            lineColor={0,255,128},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-64,19},{-28,-17}},
            lineColor={0,255,128},
            lineThickness=1,
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid)}));
  end BaseClasses;

  package Pipes

    model Pipe
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.Pipes.Pipe(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium) annotation(IconMap(primitivesVisible=false));

      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                   Rectangle(
              extent={{-100,30},{100,-30}},
              lineColor={0,255,128},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)),
                  Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Pipe;

    model AdmiLouver
      extends Pipe annotation(IconMap(primitivesVisible=false));
      annotation (Icon(graphics={                                             Line(
              points={{46,100},{46,-100},{-54,-100},{-14,-60},{-54,-60},{-14,-20},{-54,-20},{-14,20},{-54,20},{-14,60},{-54,60},{-14,100},{46,100}},
              color={0,255,128},
              thickness=0.5), Polygon(
              points={{40,96},{40,-96},{-44,-96},{-4,-56},{-44,-56},{-4,-16},{-44,-16},{-4,24},{-44,24},{-4,64},{-44,64},{-12,96},{40,96}},
              lineColor={0,255,128},
              fillColor={170,255,213},
              fillPattern=FillPattern.Solid)}));
    end AdmiLouver;

    model HeatLoss
      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.Pipes.HeatLoss(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium)  annotation(IconMap(primitivesVisible=false));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                   Rectangle(
              extent={{-100,30},{100,-30}},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None,
              lineColor={0,255,128}),
            Line(
              points={{-14,50},{-14,50},{-24,40},{-10,30},{-24,16},{-14,8}},
              color={238,46,47},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{0,50},{0,50},{-10,40},{4,30},{-10,16},{0,8}},
              color={238,46,47},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{14,50},{14,50},{4,40},{18,30},{4,16},{14,8}},
              color={238,46,47},
              smooth=Smooth.Bezier,
              thickness=0.5)}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
    end HeatLoss;

    model Leak

      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
      extends Partial.Pipes.Leak(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoHFlowModel flow_model,
        redeclare MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor flow_sensor,
        redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));
      annotation (Icon(graphics={Rectangle(
              extent={{-100,30},{100,-30}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
                                Rectangle(
              extent={{-100,40},{0,-40}},
              lineColor={0,255,128},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid),
                                 Rectangle(
              extent={{0,40},{100,-40}},
              lineColor={0,255,128},
              fillColor={170,255,213},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{12,16},{36,6}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{8,0},{24,-6}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{36,2},{60,-6}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{56,10},{80,2}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{60,-6},{84,-14}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{18,-12},{42,-20}},
              lineColor={85,170,255},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid)}));
    end Leak;

    model PressureCut
      extends RefMoistAir.BaseClasses.IsoHFlowModel
                                                 annotation(IconMap(primitivesVisible=false));
      annotation (Icon(graphics={Rectangle(
              extent={{-100,30},{100,-30}},
              lineColor={0,255,128},
              fillColor={0,255,128},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-40,-60},{0,60}},
              color={0,0,0},
              thickness=1),
            Line(
              points={{0,-60},{40,60}},
              color={0,0,0},
              thickness=1)}));
    end PressureCut;
  annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),        Rectangle(
            extent={{-48,33},{48,-37}},
            lineColor={0,255,128},
            fillColor={170,255,213},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-62,11},{-36,-15}},
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
            lineColor={0,255,128}),
          Rectangle(
            extent={{36,10},{60,-14}},
            lineColor={0,255,128},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Pipes;

  package Machines
    model InletGuideVanes

      import MetroscopeModelingLibrary.Utilities.Units;

      Units.PositiveVolumeFlowRate Qv;

      RefMoistAir.BaseClasses.IsoPHFlowModel isoPHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      RefMoistAir.Connectors.Inlet C_in annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
      RefMoistAir.Connectors.Outlet C_out annotation (Placement(transformation(extent={{40,-10},{60,10}}), iconTransformation(extent={{40,-10},{60,10}})));
    equation

      Qv = isoPHFlowModel.Qv_in;

      connect(isoPHFlowModel.C_in, C_in) annotation (Line(points={{-10,0},{-50,0},{-50,0}}, color={95,95,95}));
      connect(isoPHFlowModel.C_out, C_out) annotation (Line(points={{10,0},{30,0},{30,0},{50,0}}, color={95,95,95}));
      annotation (Icon(graphics={
            Polygon(
              points={{-36,80},{36,80},{40,76},{40,-76},{36,-80},{-36,-80},{-40,-76},{-40,76},{-36,80}},
              lineColor={0,255,128},
              fillColor={170,255,213},
              fillPattern=FillPattern.Solid,
              smooth=Smooth.Bezier,
              lineThickness=1),
            Polygon(
              points={{53,26},{-13,52},{-33,64},{-29,44},{53,26}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              smooth=Smooth.Bezier),
            Polygon(
              points={{53,-6},{-13,20},{-33,32},{-29,12},{53,-6}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              smooth=Smooth.Bezier),
            Polygon(
              points={{53,-38},{-13,-12},{-33,0},{-29,-20},{53,-38}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              smooth=Smooth.Bezier),
            Polygon(
              points={{53,-68},{-13,-42},{-33,-30},{-29,-50},{53,-68}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              smooth=Smooth.Bezier)}));
    end InletGuideVanes;

    model AirCompressor

      extends MetroscopeModelingLibrary.Partial.BaseClasses.FlowModel(
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
        redeclare package Medium = RefMoistAirMedium,
        Q_0 = 500, rho_0 = 1) annotation (IconMap(primitivesVisible=false));

      package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      Inputs.InputReal tau(start=15, min = 1) "Compression rate";
      Inputs.InputReal eta_is(start=0.8, min=0, max=1) "Nominal isentropic efficiency";

      Units.SpecificEnthalpy h_is(start=1e6) "Isentropic compression outlet enthalpy";
      RefMoistAirMedium.ThermodynamicState state_is "Isentropic compression outlet thermodynamic state";
      Real Q_reduced "Compressor reduced mass flow";

      // Failure modes
      parameter Boolean faulty = false;
      Units.Percentage eta_is_decrease(min = 0, max=100) "percentage decrease of eta_is";
      Units.Percentage tau_decrease(min = 0, max=100) "percentage decrease of tau";

      // Indicators
      Units.MassFraction x_liq_in; // Liquid water mass fraction at the inlet
      Units.MassFraction x_liq_out; // Liquid water mass fraction at the outlet
      Real relative_humidity_in; // Relative humidity at the inlet
      Real relative_humidity_out; // Relative humidity at the outlet


      Power.Connectors.Inlet C_W_in annotation (Placement(transformation(extent={{90,50},{110,70}}),  iconTransformation(extent={{90,50},{110,70}})));
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
      state_is =  Medium.setState_psX(P_out, Medium.specificEntropy(state_in), Xi);
      h_is = Medium.specificEnthalpy(state_is);

      /* Output variable */
      Q_reduced = Q * sqrt(T_in) / P_in;

      /* Indicators */
      x_liq_in = Medium.massFractionWaterNonVapor(state_in);
      x_liq_out = Medium.massFractionWaterNonVapor(state_out);
      assert(x_liq_out < 1e-10, "Condensed water at the outlet of the compressor", AssertionLevel.warning);
      relative_humidity_in = Medium.relativeHumidity(state_in);
      relative_humidity_out = Medium.relativeHumidity(state_out);


      annotation (
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-80},{100,80}},
            grid={2,2})),
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-80},{100,80}},
            grid={2,2}), graphics={Polygon(
              points={{100,26},{100,14},{100,-14},{100,-26},{80,-32},{-80,-60},{-100,-64},{-100,-40},{-100,40},{-100,64},{-80,60},{80,30},{100,26}},
              lineColor={0,255,128},
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
  annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Ellipse(
            extent={{-60,60},{60,-60}},
            lineColor={0,255,128},
            fillColor={170,255,213},
            fillPattern=FillPattern.Solid),
          Line(points={{-32,0},{30,0}},
          color={0,0,0},
          thickness=1),
          Line(points={{30,0},{6,20}},
          color={0,0,0},
          thickness=1),
          Line(points={{30,0},{6,-20}},
          color={0,0,0},
          thickness=1),
          Rectangle(
            extent={{-76,13},{-50,-13}},
            lineColor={0,255,128},
            fillColor={0,255,128},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{50,12},{74,-12}},
            lineColor={0,255,128},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end Machines;

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          fillColor={0,255,128},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}),
           Documentation(info="<html>
  <p>Licensed by Metroscope under the Modelica License 2 </p>
<p>Copyright © 2023, Metroscope.</p>
<p>This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the Modelica License 2. </p>
</html>"));

end RefMoistAir;
