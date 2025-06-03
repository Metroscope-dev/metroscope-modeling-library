within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Condenser
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  parameter Inputs.InputHeight water_height=1;
  parameter Units.Area S = 50000;

  parameter String QCp_max_side = "cold";

  Units.Power W;
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Pressure P_tot(start=Psat_0);
  Units.Pressure Psat(start=Psat_0, nominal=Psat_0);
  Units.Temperature Tsat(start=Tsat_0);
  Units.Pressure P_incond(start=0.0);
  Units.DifferentialPressure water_height_DP(start = water_height_DP_0);

  parameter Inputs.InputPressure P_offset=0 "Offset correction for ideal gas law";
  constant Real R(unit="J/(mol.K)") = Modelica.Constants.R "ideal gas constant";

    // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100, start=0, nominal=10); // Fouling percentage
  Real air_intake(unit="mol/m3", min=0, start=0, nominal=1e-3); // Air intake
  Units.Percentage Qv_cold_in_decrease(start=0); // Decrease of Qv_cold_in, in %

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 5000;
  parameter Units.MassFlowRate Q_hot_0 = 1000;
  parameter Units.Pressure Psat_0 = 0.05e5;
  parameter Units.Pressure P_cold_in_0 = 5e5;
  parameter Units.Pressure P_cold_out_0 = 4e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 15;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 25;
  parameter Units.Temperature T_hot_in_0 = Tsat_0;
  parameter Units.Temperature T_hot_out_0 = Tsat_0;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 0.5e5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 1e5;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 2e6;
  parameter Units.SpecificEnthalpy h_liq_sat_0 = Water.bubbleEnthalpy(Water.setSat_p(Psat_0));
  parameter Units.Temperature Tsat_0 = Water.saturationTemperature(Psat_0);
  parameter Units.DifferentialPressure water_height_DP_0 = 0.09e5;

  Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),iconTransformation(extent={{-110,-10},{-90,10}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=Psat_0, nominal=Psat_0), h_outflow(start=0.0)) annotation (Placement(transformation(extent={{-10,92},{10,112}}), iconTransformation(extent={{-10,92},{10,112}})));
  Connectors.Outlet C_hot_out(Q(start=-Q_hot_0), P(start=Psat_0+water_height_DP_0), h_outflow(start=h_liq_sat_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0))
                                                   annotation (Placement(transformation(extent={{88,-10},{108,10}}),iconTransformation(extent={{88,-10},{108,10}})));

  Pipes.FrictionPipe cold_side_pipe(
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,
    Q_0=Q_cold_0,
    T_0=T_cold_in_0,
    h_0=h_cold_in_0) annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  BaseClasses.IsoPFlowModel hot_side(
    T_in_0=Tsat_0,
    T_out_0=Tsat_0,
    h_in_0=h_hot_in_0,
    h_out_0=h_liq_sat_0,             Q_0=Q_hot_0, P_0=Psat_0) annotation (Placement(transformation(
        extent={{-24,-24},{24,24}},
        rotation=180,
        origin={0,22})));
  BaseClasses.IsoPFlowModel cold_side(
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_out_0,             Q_0=Q_cold_0,
    P_0=P_cold_out_0)                               annotation (Placement(transformation(extent={{-24,-24},{24,24}})));
  Pipes.HeightVariationPipe water_height_pipe(
    Q_0=Q_hot_0,
    P_in_0=Psat_0,
    P_out_0=Psat_0 + water_height_DP_0,
    T_0=Tsat_0,
    h_0=h_liq_sat_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,-46})));
  BaseClasses.IsoHFlowModel incondensables_in(P_in_0=Psat_0, P_out_0=Psat_0,
    Q_0=Q_hot_0,
    T_0=Tsat_0,
    h_0=h_hot_in_0)                           annotation (Placement(transformation(extent={{8,62},{28,82}})));
  BaseClasses.IsoHFlowModel incondensables_out(P_in_0=Psat_0, P_out_0=Psat_0,
    Q_0=Q_hot_0,
    T_0=Tsat_0,
    h_0=h_liq_sat_0)                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,-18})));

  Utilities.Interfaces.GenericReal Qv_cold_in annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-110,50}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-110,80})));
  Utilities.Interfaces.GenericReal Kfr_cold annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-110,80}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-110,40})));
  Utilities.Interfaces.GenericReal Kth annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-64,110})));
  Utilities.Interfaces.GenericReal C_incond annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={64,110})));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
    air_intake=0;
    Qv_cold_in_decrease=0;
  end if;

  // Definitions
  Q_cold = cold_side.Q;
  T_cold_in = cold_side.T_in;
  T_cold_out = cold_side.T_out;
  cold_side.Qv = Qv_cold_in * (1 - Qv_cold_in_decrease/100);

  Q_hot = hot_side.Q;
  T_hot_in = hot_side.T_in;
  T_hot_out = hot_side.T_out;

  cold_side.W = W;
  P_tot = incondensables_in.P_in;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Pressure losses
  cold_side_pipe.Kfr = Kfr_cold;
  water_height_pipe.delta_z = - water_height;
  water_height_pipe.DP = water_height_DP;

  // Incondensables
  P_incond = P_offset + R * (C_incond + air_intake) * Tsat;  // Ideal gaz law
  /* According to Dalton law, incondensable pressure is substracted first, 
  then water is condensed, then incondensable pressure is added again. */
  incondensables_in.DP = - P_incond;
  incondensables_out.DP = + P_incond;

  // Saturated vapor admission check
  assert(T_hot_in - Tsat < 0.1, "The steam admitted in the condenser in superheated", AssertionLevel.warning);

  // Condensation
  Psat = hot_side.P_in;
  Tsat =  Water.saturationTemperature(Psat);
  hot_side.h_out = Water.bubbleEnthalpy(Water.setSat_p(Psat));

  // Heat Exchange
  0 = Tsat - T_cold_out - (Tsat - T_cold_in)*exp(Kth*(1-fouling/100)*S*((T_cold_in - T_cold_out)/W));

  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(
      points={{-60,0},{-24,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side.C_out, C_cold_out) annotation (Line(
      points={{24,0},{62,0},{62,0},{98,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{-80,0},{-100,0}},
      color={28,108,200},
      thickness=1));
  connect(water_height_pipe.C_out, C_hot_out) annotation (Line(
      points={{-40,-56},{-40,-60},{0,-60},{0,-80}},
      color={238,46,47},
      thickness=1));
  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{98,0},{98,0},{96,0},{96,0},{98,0},{98,0}},
                                                 color={28,108,200}));
  connect(hot_side.C_in, incondensables_in.C_out) annotation (Line(
      points={{24,22},{34,22},{34,72},{28,72}},
      color={238,46,47},
      thickness=1));
  connect(incondensables_in.C_in, C_hot_in) annotation (Line(
      points={{8,72},{0,72},{0,102}},
      color={238,46,47},
      thickness=1));
  connect(hot_side.C_out, incondensables_out.C_in) annotation (Line(
      points={{-24,22},{-40,22},{-40,-8}},
      color={238,46,47},
      thickness=1));
  connect(incondensables_out.C_out, water_height_pipe.C_in) annotation (Line(
      points={{-40,-28},{-40,-36}},
      color={238,46,47},
      thickness=1));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}}),                                   graphics={
        Rectangle(
          extent={{-100,100},{100,-46}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,8},{-36,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-80},{100,-46}},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{100,-80},{100,100},{-100,100},{-100,-80},{100,-80}},
          lineColor={0,0,255}),
        Ellipse(
          extent={{-62,18},{-58,12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,32},{-36,26}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,20},{-16,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-8,36},{-4,30}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,32},{-16,26}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,50},{-36,44}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,20},{14,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,38},{16,30}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,28},{36,22}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,42},{44,36}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{48,22},{52,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,50},{-12,44}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{4,52},{8,46}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,58},{28,52}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,52},{54,46}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-56,38},{-52,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,54},{-56,48}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,-30},{-56,-36}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-42,-36},{-38,-42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-24,-28},{-20,-34}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,-34},{0,-40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-30},{18,-36}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,-32},{36,-38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,-30},{54,-36}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,-32},{68,-38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,10},{36,4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,22},{68,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,12},{0,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-18,0},{-14,-6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,-4},{24,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,-10},{2,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-38,-14},{-34,-20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,-6},{-54,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,-10},{46,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{54,-2},{58,-8}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,-14},{64,-20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,-14},{14,-20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-78,38},{-74,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,14},{-70,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-76,-14},{-72,-20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-70,-26},{-66,-32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,36},{64,30}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,48},{74,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,50},{-70,44}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-100,-4},{100,-4}},
          color={0,0,255},
          thickness=1),
        Line(
          points={{-100,-16},{100,-16}},
          color={0,0,255},
          thickness=1),
        Line(
          points={{-100,-28},{100,-28}},
          color={0,0,255},
          thickness=1),
        Line(
          points={{-100,32},{100,32}},
          color={0,0,255},
          thickness=1),
        Line(
          points={{-100,20},{100,20}},
          color={0,0,255},
          thickness=1),
        Line(
          points={{-100,8},{100,8}},
          color={0,0,255},
          thickness=1),
        Ellipse(
          extent={{-46,62},{-42,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-14,66},{-10,60}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-26,62},{-22,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-46,80},{-42,74}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,68},{10,60}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{34,72},{38,66}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-22,80},{-18,74}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,82},{2,76}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,88},{22,82}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{44,82},{48,76}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-62,68},{-58,62}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-66,84},{-62,78}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{54,66},{58,60}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,78},{68,72}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}})));
end Condenser;
