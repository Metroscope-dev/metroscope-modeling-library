within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Condenser
  package Water = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputHeight water_height;
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputArea S;
  Units.HeatExchangeCoefficient Kth;
  Units.VolumeFlowRate Qv_cold_in(start=Q_cold_0/1e3);

  parameter String QCp_max_side = "cold";

  Units.Power W;
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Pressure P_tot(start=Psat_0, nominal=Psat_0);
  Units.Pressure Psat(start=Psat_0, nominal=Psat_0);
  Units.Temperature Tsat(start=Tsat_0);
  Units.Pressure P_incond(start=0.001e5);
  Inputs.InputReal C_incond(unit="mol/m3", min=0) "Incondensable molar concentration";
  Inputs.InputPressure P_offset(start=0) "Offset correction for ideal gas law";
  constant Real R(unit="J/(mol.K)") = Modelica.Constants.R "ideal gas constant";

    // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage
  Real air_intake(unit="mol/m3", min=0); // Air intake

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 5000;
  parameter Units.MassFlowRate Q_hot_0 = 1000;
  parameter Units.Pressure Psat_0 = 0.19e5;
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


  Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-110,30},{-90,50}}), iconTransformation(extent={{-110,30},{-90,50}})));
  Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=Psat_0, nominal=Psat_0)) annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  Connectors.Outlet C_hot_out(Q(start=Q_cold_0), P(start=Psat_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
  Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0))
                                                   annotation (Placement(transformation(extent={{90,-10},{110,10}}),iconTransformation(extent={{90,-10},{110,10}})));

  Pipes.Pipe cold_side_pipe(
    P_in_0=P_cold_in_0,
    P_out_0=P_cold_out_0,   Q_0=Q_cold_0,
    T_0=T_cold_in_0,
    h_0=h_cold_in_0)                      annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
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
  Pipes.Pipe water_height_pipe(
    Q_0=Q_hot_0,
    P_in_0=Psat_0,
    P_out_0=Psat_0,
    T_0=Tsat_0,
    h_0=h_liq_sat_0,
    h(start=2.46e5)) annotation (Placement(transformation(
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

equation

  // Failure modes
  if not faulty then
    fouling = 0;
    air_intake=0;
  end if;


  // Definitions
  Q_cold = cold_side.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side.T_out;
  Qv_cold_in = cold_side.Q / cold_side.rho;

  Q_hot = hot_side.Q;
  T_hot_in = hot_side.T_in;
  T_hot_out = water_height_pipe.T_out;

  cold_side.W = W;
  P_tot = incondensables_in.P_in;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Pressure losses
  cold_side_pipe.delta_z = 0;
  cold_side_pipe.Kfr = Kfr_cold;

  water_height_pipe.delta_z = - water_height;
  water_height_pipe.Kfr = 0;

  // Incondensables
  P_incond = P_offset + R * (C_incond + air_intake) * Tsat;  // Ideal gaz law
  /* According to Dalton law, incondensable pressure is substracted first, 
  then water is condensed, then incondensable pressure is added again. */
  incondensables_in.DP = - P_incond;
  incondensables_out.DP = + P_incond;

  // Condensation
  Psat = hot_side.P_in;
  Tsat = hot_side.T_in;
  hot_side.h_out = Water.bubbleEnthalpy(Water.setSat_T(Tsat));

  // Heat Exchange
  0 = Tsat - T_cold_out - (Tsat - T_cold_in)*exp(Kth*(1-fouling/100)*S*((T_cold_in - T_cold_out)/W));


  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(
      points={{-60,40},{-52,40},{-52,0},{-24,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side.C_out, C_cold_out) annotation (Line(
      points={{24,0},{100,0}},
      color={28,108,200},
      thickness=1));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(
      points={{-80,40},{-100,40}},
      color={28,108,200},
      thickness=1));
  connect(water_height_pipe.C_out, C_hot_out) annotation (Line(
      points={{-40,-56},{-40,-60},{0,-60},{0,-80}},
      color={238,46,47},
      thickness=1));
  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{100,0},{100,0}},   color={28,108,200}));
  connect(hot_side.C_in, incondensables_in.C_out) annotation (Line(
      points={{24,22},{34,22},{34,72},{28,72}},
      color={238,46,47},
      thickness=1));
  connect(incondensables_in.C_in, C_hot_in) annotation (Line(
      points={{8,72},{0,72},{0,100}},
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
          extent={{-100,100},{100,-20}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-40,34},{-36,28}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-86},{100,-20}},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{100,-86},{100,100},{-100,100},{-100,-86},{100,-86}},
          lineColor={0,0,255}),
        Ellipse(
          extent={{-62,44},{-58,38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,58},{-36,52}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,46},{-16,40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-8,62},{-4,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,58},{-16,52}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,76},{-36,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,46},{14,40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,64},{16,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,54},{36,48}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,68},{44,62}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{48,48},{52,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,76},{-12,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{4,78},{8,72}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,84},{28,78}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,78},{54,72}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-56,64},{-52,58}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,80},{-56,74}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,-4},{-56,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-42,-10},{-38,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-24,-2},{-20,-8}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,-8},{0,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-4},{18,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,-6},{36,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,-4},{54,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,-6},{68,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,36},{36,30}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,48},{68,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,38},{0,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-18,26},{-14,20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,22},{24,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,16},{2,10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-38,12},{-34,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,20},{-54,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,16},{46,10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{54,24},{58,18}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,12},{64,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,12},{14,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-78,64},{-74,58}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,40},{-70,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-76,12},{-72,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-70,0},{-66,-6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,62},{64,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,74},{74,68}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,76},{-70,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-100,50},{82,50},{82,36},{-88,36},{-88,22},{82,22},{82,8},{
              -88,8},{-88,-8},{100,-8}},
          color={0,0,255},
          thickness=1)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}})));
end Condenser;
