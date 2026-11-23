within MetroscopeModelingLibrary.Partial.HeatExchangers;
partial model LiqLiqHX
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  parameter Inputs.InputArea S=100;
  parameter String QCp_max_side = "unknown";
  parameter String HX_config = "shell_and_tubes_two_passes"; // Valid for U-shaped tubes. Otherwise use "monophasic_cross_current"

  Units.Power W;
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  // Indicators
  Units.DifferentialTemperature FTR(start=T_cold_out_0-T_cold_in_0) "Feedwater Temperature Rise";
  Units.DifferentialTemperature TTD(start=T_hot_in_0-T_cold_out_0) "Terminal Temperature Difference";
  Units.DifferentialTemperature DCA(start=T_hot_out_0-T_cold_in_0) "Drain Cooler Approach";
  Units.DifferentialTemperature pinch(start=min(T_hot_in_0-T_cold_out_0,T_hot_out_0-T_cold_in_0)) "Lowest temperature difference";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Pressure P_cold_in_0 = 10e5;
  parameter Units.Pressure P_cold_out_0 = 9e5;
  parameter Units.Pressure P_hot_in_0 = 5e5;
  parameter Units.Pressure P_hot_out_0 = 4e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 50;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 100;
  parameter Units.Temperature T_hot_in_0 = 273.15 + 150;
  parameter Units.Temperature T_hot_out_0 = 273.15 + 100;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 5e5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 5e5;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 5e5;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 5e5;

  Connectors.FluidInlet    C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0))
    annotation (Placement(transformation(extent={{-172,-10},{-152,10}}),
        iconTransformation(extent={{-172,-10},{-152,10}})));
  Connectors.FluidInlet    C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0))
    annotation (Placement(transformation(extent={{-10,70},{10,90}}),
        iconTransformation(extent={{-10,70},{10,90}})));
  Connectors.FluidOutlet    C_hot_out(
    Q(start=Q_cold_0),
    P(start=P_hot_out_0),
    h_outflow(start=h_hot_out_0)) annotation (Placement(transformation(extent={{
            -10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.FluidOutlet    C_cold_out(
    Q(start=-Q_cold_0),
    P(start=P_cold_out_0),
    h_outflow(start=h_cold_out_0)) annotation (Placement(transformation(extent={
            {150,-10},{170,10}}), iconTransformation(extent={{150,-10},{170,10}})));

  Partial.Pipes.FrictionPipe cold_side_pipe(
    Q_0=Q_cold_0,
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,
    h_0=h_cold_in_0,
    T_0=T_cold_in_0)
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Partial.Pipes.FrictionPipe hot_side_pipe(
    Q_0=Q_hot_0,
    P_in_0=P_hot_in_0,
    P_out_0=P_hot_out_0,
    h_0=h_hot_in_0,
    T_0=T_hot_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,52})));
  Partial.BaseClasses.IsoPFlowModel hot_side(
    Q_0=Q_hot_0,
    P_0=P_hot_out_0,
    h_in_0=h_hot_in_0,
    h_out_0=h_hot_out_0,
    T_in_0=T_hot_in_0,
    T_out_0=T_hot_out_0) annotation (Placement(transformation(
        extent={{-23,-23},{23,23}},
        rotation=180,
        origin={-1,21})));
  Partial.BaseClasses.IsoPFlowModel cold_side(
    Q_0=Q_cold_0,
    P_0=P_cold_out_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_out_0,
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_out_0)
    annotation (Placement(transformation(extent={{-26,-58},{22,-10}})));
  Power.HeatExchange.NTUHeatExchange HX(config=HX_config, QCp_max_side = QCp_max_side) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-6})));
  Utilities.Interfaces.GenericReal Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={100,84}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={100,84})));
  Utilities.Interfaces.GenericReal Kth annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-100,84}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-100,84})));
  Utilities.Interfaces.GenericReal Kfr_cold annotation (Placement(transformation(extent={{-160,36},{-168,44}}), iconTransformation(extent={{-160,36},{-168,44}})));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  // Definitions
  Q_cold = cold_side.Q;
  Q_hot = hot_side.Q;
  T_cold_in = cold_side.T_in;
  T_cold_out = cold_side.T_out;
  T_hot_in = hot_side.T_in;
  T_hot_out = hot_side.T_out;
  cold_side.W = W;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Power Exchange
  HX.W = W;
  HX.Kth =  Kth*(1-fouling/100);
  HX.S = S;
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.Cp_cold = Medium.specificHeatCapacityCp(cold_side.state_in);
  HX.Cp_hot = Medium.specificHeatCapacityCp(hot_side.state_in);

  // Indicators
  FTR = T_cold_out - T_cold_in;
  TTD = T_hot_in - T_cold_out;
  DCA = T_hot_out - T_cold_in;
  pinch = min(TTD, DCA);
  assert(pinch > 0, "A negative pinch is reached", AssertionLevel.warning); // Ensure a positive pinch
  assert(pinch > 1 or pinch < 0,  "A very low pinch (<1) is reached", AssertionLevel.warning); // Ensure a sufficient pinch

  connect(cold_side_pipe.Kfr, Kfr_cold) annotation (Line(points={{-130,4},{-130,32},{-164,32},{-164,40}}, color={0,0,127}));
  connect(hot_side_pipe.Kfr, Kfr_hot) annotation (Line(points={{4,52},{24,52},{24,84},{100,84}},color={0,0,127}));
  connect(C_cold_in, cold_side_pipe.C_in)
    annotation (Line(points={{-162,0},{-140,0}}, color={95,95,95}));
  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{-120,0},
          {-86,0},{-86,-34},{-26,-34}}, color={0,0,0}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{22,-34},{74,-34},
          {74,-36},{120,-36},{120,0},{160,0}}, color={0,0,0}));
  connect(hot_side_pipe.C_in, C_hot_in)
    annotation (Line(points={{0,62},{0,80}}, color={95,95,95}));
  connect(hot_side_pipe.C_out, hot_side.C_in) annotation (Line(points={{0,42},{8,
          42},{8,38},{66,38},{66,21},{22,21}}, color={0,0,0}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-24,21},{-42,21},
          {-42,20},{-56,20},{-56,-80},{0,-80}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -80},{160,80}},
        initialScale=0.3,
        grid={2,2}),         graphics={
        Polygon(
          points={{-160,80},{-160,0},{-160,-80},{-60,-80},{60,-80},{160,-80},{160,
              0},{160,80},{60,80},{-60,80},{-160,80}},
          lineColor={128,255,0},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-124,62},{-124,40},{-124,-40},{-124,-60},{-104,-60},{-12,-60},
              {94,-60},{116,-60},{116,-38},{116,43.0566},{116,62},{96,62},{-16,62},
              {-104,62},{-124,62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-130,50},{126,-42}},
          lineColor={0,140,72},
          lineThickness=1),
        Rectangle(
          extent={{-126,42},{124,-34}},
          lineColor={0,140,72},
          lineThickness=1),
        Rectangle(
          extent={{-130,34},{126,-26}},
          lineColor={0,140,72},
          lineThickness=1),
        Rectangle(
          extent={{116,52},{142,-54}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-148,54},{-124,-52}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={0,127,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,140,72},
          radius=0)}),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},{160,80}},
        initialScale=0.3,
        grid={2,2})));
end LiqLiqHX;
